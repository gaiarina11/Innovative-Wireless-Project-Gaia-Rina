/* Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Single-link Channel Sounding anchor with USB distance reporting.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/console/console.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/ras.h>
#include <dk_buttons_and_leds.h>

#include "distance_estimation.h"

LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#ifndef ANCHOR_ID
#define ANCHOR_ID 0
#endif

BUILD_ASSERT(ANCHOR_ID >= 0 && ANCHOR_ID <= 2, "ANCHOR_ID must be 0, 1 or 2");

#define CON_STATUS_LED DK_LED1
#define TARGET_DEVICE_NAME "CS_TAG"
#define CS_CONFIG_ID 0
#define NUM_MODE_0_STEPS 3
#define PROCEDURE_COUNTER_NONE (-1)

/* Only one anchor is allowed to run CS at a time. The host grants the token
 * over UART. A range, instead of a fixed procedure interval, lets the
 * controller choose a radio slot that does not collide with the tag's other
 * two ACL links.
 */
/* 100 ms gives the reflector controller enough room for three ACL events and
 * the commanded Mode 3 subevent. The CS window stays close to the grant so a
 * retry does not make the host snapshot stale.
 */
#define ACL_INTERVAL_UNITS 80
#define CS_MIN_INTERVAL_EVENTS 1
#define CS_MAX_INTERVAL_EVENTS 4
#define MEASUREMENT_MAX_ATTEMPTS 2
#define MEASUREMENT_RETRY_DELAY K_MSEC(100)
#define HOST_COMMAND_MAX_LEN 32
#define COMMAND_THREAD_STACK_SIZE 1024
#define COMMAND_THREAD_PRIORITY 7

#define LOCAL_PROCEDURE_MEM                                                                        \
	((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) +                \
	 (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_disconnected, 0, 1);
static K_SEM_DEFINE(sem_security, 0, 1);
static K_SEM_DEFINE(sem_mtu_exchange_done, 0, 1);
static K_SEM_DEFINE(sem_discovery_done, 0, 1);
static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_procedure_enabled, 0, 1);
static K_SEM_DEFINE(sem_procedure_done, 0, 1);
static K_SEM_DEFINE(sem_rd_ready, 0, 1);
static K_SEM_DEFINE(sem_rd_complete, 0, 1);
static K_SEM_DEFINE(sem_measure_request, 0, 1);
static K_SEM_DEFINE(sem_console_initialized, 0, 1);

static struct bt_conn *connection;
static atomic_t connection_active;
static int connection_status;
static int security_status;
static int mtu_status;
static int discovery_status;
static int rd_complete_status;
static bool procedure_is_enabled;
static int procedure_result_status;

static struct bt_conn_le_cs_capabilities remote_caps;
static uint8_t num_antenna_paths;
static const enum bt_conn_le_cs_rtt_type selected_rtt_type =
	BT_CONN_LE_CS_RTT_TYPE_AA_ONLY;

NET_BUF_SIMPLE_DEFINE_STATIC(local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(peer_steps, BT_RAS_PROCEDURE_MEM);

static int32_t most_recent_peer_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
static bool distance_estimation_in_progress;
static bool local_procedure_valid;
static uint32_t distance_sequence;
static atomic_t measurement_ready;
static atomic_t measurement_busy;

static void command_thread(void *unused1, void *unused2, void *unused3);

K_THREAD_DEFINE(command_thread_id, COMMAND_THREAD_STACK_SIZE, command_thread, NULL, NULL, NULL,
		COMMAND_THREAD_PRIORITY, 0, 0);

static void report_cs_status(const char *state, int code)
{
	if (code == 0) {
		printk("CS_STATUS:ANCHOR:%d|STATE:%s\n", ANCHOR_ID, state);
	} else {
		printk("CS_STATUS:ANCHOR:%d|STATE:%s|CODE:%d\n", ANCHOR_ID, state, code);
	}
}

static void process_host_command(const char *command)
{
	if (strcmp(command, "CS_MEASURE") == 0) {
		if (!atomic_get(&measurement_ready)) {
			report_cs_status("NOT_READY", -EAGAIN);
		} else if (atomic_get(&measurement_busy)) {
			report_cs_status("BUSY", -EBUSY);
		} else {
			k_sem_give(&sem_measure_request);
			report_cs_status("QUEUED", 0);
		}
	} else if (strcmp(command, "CS_PING") == 0) {
		if (atomic_get(&measurement_busy)) {
			report_cs_status("BUSY", 0);
		} else if (atomic_get(&measurement_ready)) {
			report_cs_status("READY", 0);
		} else if (atomic_get(&connection_active)) {
			report_cs_status("CONFIGURING", 0);
		} else {
			report_cs_status("DISCONNECTED", 0);
		}
	} else {
		report_cs_status("BAD_COMMAND", -EINVAL);
	}
}

static void command_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	char command[HOST_COMMAND_MAX_LEN];
	size_t length = 0;

	k_sem_take(&sem_console_initialized, K_FOREVER);
	while (true) {
		int value = console_getchar();

		if (value < 0) {
			continue;
		}

		char character = (char)value;

		if (character == '\r' || character == '\n') {
			if (length > 0) {
				command[length] = '\0';
				process_host_command(command);
				length = 0;
			}
		} else if (length < sizeof(command) - 1) {
			command[length++] = character;
		} else {
			length = 0;
			report_cs_status("BAD_COMMAND", -EMSGSIZE);
		}
	}
}

static bool is_active_connection(struct bt_conn *conn)
{
	return atomic_get(&connection_active) && connection == conn;
}

static void reset_measurement_buffers(void)
{
	net_buf_simple_reset(&local_steps);
	net_buf_simple_reset(&peer_steps);
	distance_estimation_in_progress = false;
	local_procedure_valid = false;
}

static void reset_measurement_state(void)
{
	reset_measurement_buffers();
	most_recent_peer_ranging_counter = PROCEDURE_COUNTER_NONE;
	most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
	dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
	k_sem_reset(&sem_procedure_done);
	k_sem_reset(&sem_rd_ready);
	k_sem_reset(&sem_rd_complete);
	procedure_result_status = -EINPROGRESS;
}

static int subevent_abort_errno(enum bt_conn_le_cs_subevent_abort_reason reason)
{
	switch (reason) {
	case BT_CONN_LE_CS_SUBEVENT_ABORT_SCHED_CONFLICT:
		return -EAGAIN;
	case BT_CONN_LE_CS_SUBEVENT_ABORT_NO_CS_SYNC:
		return -ENOLINK;
	case BT_CONN_LE_CS_SUBEVENT_ABORT_REQUESTED:
		return -ECANCELED;
	default:
		return -EIO;
	}
}

static int procedure_abort_errno(enum bt_conn_le_cs_procedure_abort_reason reason)
{
	switch (reason) {
	case BT_CONN_LE_CS_PROCEDURE_ABORT_REQUESTED:
		return -ECANCELED;
	case BT_CONN_LE_CS_PROCEDURE_ABORT_TOO_FEW_CHANNELS:
		return -ENODATA;
	default:
		return -EIO;
	}
}

static void subevent_result_cb(struct bt_conn *conn,
			       struct bt_conn_le_cs_subevent_result *result)
{
	if (!is_active_connection(conn)) {
		return;
	}

	uint16_t procedure_counter = result->header.procedure_counter;

	if (distance_estimation_in_progress) {
		dropped_ranging_counter = procedure_counter;
		return;
	}

	if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
		procedure_result_status =
			subevent_abort_errno(result->header.subevent_abort_reason);
		LOG_WRN("Subevent %u aborted: reason=%u step=%u (err %d)", procedure_counter,
			result->header.subevent_abort_reason, result->header.abort_step,
			procedure_result_status);
		dropped_ranging_counter = procedure_counter;
		net_buf_simple_reset(&local_steps);
	} else if (dropped_ranging_counter != procedure_counter && result->step_data_buf) {
		if (result->step_data_buf->len <= net_buf_simple_tailroom(&local_steps)) {
			uint16_t len = result->step_data_buf->len;
			uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);

			net_buf_simple_add_mem(&local_steps, step_data, len);
		} else {
			LOG_ERR("Not enough memory for local step data (%u > %u)",
				local_steps.len + result->step_data_buf->len, local_steps.size);
			net_buf_simple_reset(&local_steps);
			dropped_ranging_counter = procedure_counter;
		}
	}

	if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		local_procedure_valid = dropped_ranging_counter != procedure_counter;
		if (local_procedure_valid) {
			procedure_result_status = 0;
			num_antenna_paths = result->header.num_antenna_paths;
			most_recent_local_ranging_counter = procedure_counter;
			distance_estimation_in_progress = true;
		} else {
			LOG_WRN("Dropping incomplete procedure %u", procedure_counter);
		}
		dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
		k_sem_give(&sem_procedure_done);
	} else if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
		if (procedure_result_status == -EINPROGRESS) {
			procedure_result_status =
				procedure_abort_errno(result->header.procedure_abort_reason);
		}
		LOG_WRN("Procedure %u aborted: procedure_reason=%u subevent_reason=%u "
			"step=%u (err %d)", procedure_counter,
			result->header.procedure_abort_reason,
			result->header.subevent_abort_reason, result->header.abort_step,
			procedure_result_status);
		net_buf_simple_reset(&local_steps);
		local_procedure_valid = false;
		distance_estimation_in_progress = false;
		dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
		k_sem_give(&sem_procedure_done);
	}
}

static void ranging_data_get_complete_cb(struct bt_conn *conn, uint16_t ranging_counter, int err)
{
	if (!is_active_connection(conn)) {
		return;
	}

	rd_complete_status = err;
	if (err) {
		LOG_ERR("Ranging data %u transfer failed (err %d)", ranging_counter, err);
	} else {
		LOG_DBG("Ranging data %u transfer complete", ranging_counter);
	}
	k_sem_give(&sem_rd_complete);
}

static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	if (!is_active_connection(conn)) {
		return;
	}

	most_recent_peer_ranging_counter = ranging_counter;
	k_sem_give(&sem_rd_ready);
}

static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	if (is_active_connection(conn)) {
		LOG_WRN("Peer ranging data %u overwritten", ranging_counter);
	}
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	ARG_UNUSED(params);

	if (!is_active_connection(conn)) {
		return;
	}

	mtu_status = err ? -EIO : 0;
	if (err) {
		LOG_ERR("MTU exchange failed (err %u)", err);
	} else {
		LOG_INF("MTU exchange success (%u)", bt_gatt_get_mtu(conn));
	}
	k_sem_give(&sem_mtu_exchange_done);
}

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	ARG_UNUSED(context);

	struct bt_conn *conn = bt_gatt_dm_conn_get(dm);
	int err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);

	discovery_status = err;
	if (err) {
		LOG_ERR("RAS RREQ handle assignment failed (err %d)", err);
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_ERR("Discovery data release failed (err %d)", err);
		if (discovery_status == 0) {
			discovery_status = err;
		}
	}

	k_sem_give(&sem_discovery_done);
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(context);

	LOG_ERR("Ranging Service not found");
	discovery_status = -ENOENT;
	k_sem_give(&sem_discovery_done);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(context);

	LOG_ERR("Service discovery failed (err %d)", err);
	discovery_status = err ? err : -EIO;
	k_sem_give(&sem_discovery_done);
}

static struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	if (!is_active_connection(conn)) {
		return;
	}

	security_status = err ? -EACCES : 0;
	if (err) {
		LOG_ERR("Security setup failed (level %u, err %d: %s)", level, err,
			bt_security_err_to_str(err));
	} else {
		LOG_INF("Security level %u established", level);
	}
	k_sem_give(&sem_security);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(param);

	/* Keep the interval used to dimension the CS procedure period. */
	return false;
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (err) {
		LOG_ERR("Connection to %s failed (err 0x%02x)", addr, err);
		connection_status = -ECONNREFUSED;
		k_sem_give(&sem_connected);
		return;
	}

	if (connection != NULL) {
		LOG_WRN("Rejecting unexpected second connection");
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	connection = bt_conn_ref(conn);
	atomic_set(&connection_active, 1);
	connection_status = 0;
	LOG_INF("Connected to CS_TAG at %s", addr);
	dk_set_led_on(CON_STATUS_LED);
	k_sem_give(&sem_connected);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	LOG_WRN("Disconnected (reason 0x%02x)", reason);
	atomic_clear(&measurement_ready);
	atomic_clear(&measurement_busy);
	report_cs_status("DISCONNECTED", reason);

	if (connection == conn) {
		atomic_clear(&connection_active);
		bt_conn_unref(connection);
		connection = NULL;
	}

	dk_set_led_off(CON_STATUS_LED);
	k_sem_give(&sem_disconnected);
}

static void remote_capabilities_cb(struct bt_conn *conn,
				   struct bt_conn_le_cs_capabilities *params)
{
	if (!is_active_connection(conn)) {
		return;
	}

	if (params != NULL) {
		remote_caps = *params;
		LOG_INF("CS capabilities: antennas=%u, sync_2m=%u", params->num_antennas_supported,
			params->cs_sync_2m_phy_supported);
	}
	k_sem_give(&sem_remote_capabilities_obtained);
}

static void config_created_cb(struct bt_conn *conn, struct bt_conn_le_cs_config *config)
{
	if (is_active_connection(conn)) {
		LOG_INF("CS configuration %u created", config->id);
		k_sem_give(&sem_config_created);
	}
}

static void cs_security_enabled_cb(struct bt_conn *conn)
{
	if (is_active_connection(conn)) {
		LOG_INF("CS security enabled");
		k_sem_give(&sem_cs_security_enabled);
	}
}

static void procedure_enabled_cb(struct bt_conn *conn,
				 struct bt_conn_le_cs_procedure_enable_complete *params)
{
	if (!is_active_connection(conn)) {
		return;
	}

	procedure_is_enabled = params->state == BT_CONN_LE_CS_PROCEDURES_ENABLED;
	LOG_INF("CS procedures %s", procedure_is_enabled ? "enabled" : "disabled");
	k_sem_give(&sem_procedure_enabled);
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_param_req = le_param_req,
	.security_changed = security_changed,
	.le_cs_remote_capabilities_available = remote_capabilities_cb,
	.le_cs_config_created = config_created_cb,
	.le_cs_security_enabled = cs_security_enabled_cb,
	.le_cs_procedure_enabled = procedure_enabled_cb,
	.le_cs_subevent_data_available = subevent_result_cb,
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	ARG_UNUSED(filter_match);

	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
	LOG_INF("CS_TAG matched at %s (connectable=%u)", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	ARG_UNUSED(device_info);

	LOG_ERR("Connection creation failed");
	connection_status = -EIO;
	k_sem_give(&sem_connected);
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
	ARG_UNUSED(device_info);
	ARG_UNUSED(conn);

	LOG_INF("Connecting to CS_TAG");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

static int scan_init(void)
{
	struct bt_scan_init_param init_param = {
		.scan_param = NULL,
		.conn_param = BT_LE_CONN_PARAM(ACL_INTERVAL_UNITS, ACL_INTERVAL_UNITS, 0, 400),
		.connect_if_match = true,
	};

	bt_scan_init(&init_param);
	bt_scan_cb_register(&scan_cb);

	int err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_RANGING_SERVICE);

	if (err) {
		LOG_ERR("Cannot add Ranging Service scan filter (err %d)", err);
		return err;
	}

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, TARGET_DEVICE_NAME);
	if (err) {
		LOG_ERR("Cannot add CS_TAG name scan filter (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER | BT_SCAN_NAME_FILTER, true);
	if (err) {
		LOG_ERR("Cannot enable scan filters (err %d)", err);
	}

	return err;
}

static int take_setup_sem(struct k_sem *sem, k_timeout_t timeout, const char *operation)
{
	int err = k_sem_take(sem, timeout);

	if (err) {
		LOG_ERR("Timeout waiting for %s", operation);
		return -ETIMEDOUT;
	}

	return 0;
}

static int configure_connection(struct bt_conn *conn)
{
	int err;

	k_sem_reset(&sem_security);
	security_status = -ETIMEDOUT;
	err = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (err) {
		LOG_ERR("Cannot request link security (err %d)", err);
		return err;
	}
	err = take_setup_sem(&sem_security, K_SECONDS(10), "link security");
	if (err || security_status) {
		return err ? err : security_status;
	}

	static struct bt_gatt_exchange_params mtu_params;
	mtu_params.func = mtu_exchange_cb;
	k_sem_reset(&sem_mtu_exchange_done);
	mtu_status = -ETIMEDOUT;
	err = bt_gatt_exchange_mtu(conn, &mtu_params);
	if (err) {
		LOG_ERR("Cannot start MTU exchange (err %d)", err);
		return err;
	}
	err = take_setup_sem(&sem_mtu_exchange_done, K_SECONDS(10), "MTU exchange");
	if (err || mtu_status) {
		return err ? err : mtu_status;
	}

	k_sem_reset(&sem_discovery_done);
	discovery_status = -ETIMEDOUT;
	err = bt_gatt_dm_start(conn, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
	if (err) {
		LOG_ERR("Cannot start Ranging Service discovery (err %d)", err);
		return err;
	}
	err = take_setup_sem(&sem_discovery_done, K_SECONDS(10), "service discovery");
	if (err || discovery_status) {
		return err ? err : discovery_status;
	}

	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = true,
		.enable_reflector_role = false,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	err = bt_le_cs_set_default_settings(conn, &default_settings);
	if (err) {
		LOG_ERR("Cannot set default CS settings (err %d)", err);
		return err;
	}

	err = bt_ras_rreq_rd_overwritten_subscribe(conn, ranging_data_overwritten_cb);
	if (err) {
		LOG_ERR("Cannot subscribe to ranging-data-overwritten (err %d)", err);
		return err;
	}
	err = bt_ras_rreq_rd_ready_subscribe(conn, ranging_data_ready_cb);
	if (err) {
		LOG_ERR("Cannot subscribe to ranging-data-ready (err %d)", err);
		return err;
	}
	err = bt_ras_rreq_on_demand_rd_subscribe(conn);
	if (err) {
		LOG_ERR("Cannot subscribe to on-demand ranging data (err %d)", err);
		return err;
	}
	err = bt_ras_rreq_cp_subscribe(conn);
	if (err) {
		LOG_ERR("Cannot subscribe to RAS control point (err %d)", err);
		return err;
	}

	memset(&remote_caps, 0, sizeof(remote_caps));
	k_sem_reset(&sem_remote_capabilities_obtained);
	err = bt_le_cs_read_remote_supported_capabilities(conn);
	if (err) {
		LOG_ERR("Cannot read remote CS capabilities (err %d)", err);
		return err;
	}
	err = take_setup_sem(&sem_remote_capabilities_obtained, K_SECONDS(10),
			     "remote CS capabilities");
	if (err) {
		return err;
	}

	struct bt_le_cs_create_config_params config_params = {
		.id = CS_CONFIG_ID,
		/* Mode 3 carries an RTT exchange and PBR tones in every main step.
		 * Keeping the sub-mode unused makes every reported procedure directly
		 * comparable and lets the host inspect both estimates from one token.
		 */
		.main_mode_type = BT_CONN_LE_CS_MAIN_MODE_3,
		.sub_mode_type = BT_CONN_LE_CS_SUB_MODE_UNUSED,
		/* Twelve Mode 3 steps normally yield about 24 PBR tone samples and
		 * twelve RTT timings. More importantly, their short subevents can be
		 * scheduled between the tag's three active ACL connection events.
		 */
		.min_main_mode_steps = 12,
		.max_main_mode_steps = 12,
		.main_mode_repetition = 0,
		.mode_0_steps = NUM_MODE_0_STEPS,
		.role = BT_CONN_LE_CS_ROLE_INITIATOR,
		.rtt_type = selected_rtt_type,
		.cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY,
		.channel_map_repetition = 5,
		.channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
		.ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT,
		.ch3c_jump = 2,
	};
	/* AA-only is intentional for the three-link topology: the shorter CS_SYNC
	 * exchange has substantially more scheduling and RF margin than the
	 * 96-bit sounding packet while still providing an independent RTT value.
	 */
	LOG_INF("CS Mode 3: PBR+RTT, RTT type=AA-only (remote sounding N=%u)",
		 remote_caps.rtt_sounding_n);

	bt_le_cs_set_valid_chmap_bits(config_params.channel_map);
	k_sem_reset(&sem_config_created);
	err = bt_le_cs_create_config(conn, &config_params,
				     BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
	if (err) {
		LOG_ERR("Cannot create CS configuration (err %d)", err);
		return err;
	}
	err = take_setup_sem(&sem_config_created, K_SECONDS(10), "CS configuration");
	if (err) {
		return err;
	}

	k_sem_reset(&sem_cs_security_enabled);
	err = bt_le_cs_security_enable(conn);
	if (err) {
		LOG_ERR("Cannot enable CS security (err %d)", err);
		return err;
	}
	err = take_setup_sem(&sem_cs_security_enabled, K_SECONDS(10), "CS security");
	if (err) {
		return err;
	}

	const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
		.config_id = CS_CONFIG_ID,
		.max_procedure_len = 64,
		.min_procedure_interval = CS_MIN_INTERVAL_EVENTS,
		.max_procedure_interval = CS_MAX_INTERVAL_EVENTS,
		.max_procedure_count = 1,
		.min_subevent_len = 5000,
		.max_subevent_len = 10000,
		.tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_INDEX_ONE,
		/* Use 1M for CS_SYNC reliability with three simultaneous ACL links. */
		.phy = BT_LE_CS_PROCEDURE_PHY_1M,
		.tx_power_delta = 0x80,
		.preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
		.snr_control_initiator = BT_LE_CS_INITIATOR_SNR_CONTROL_NOT_USED,
		.snr_control_reflector = BT_LE_CS_REFLECTOR_SNR_CONTROL_NOT_USED,
	};

	err = bt_le_cs_set_procedure_parameters(conn, &procedure_params);
	if (err) {
		LOG_ERR("Cannot set CS procedure parameters (err %d)", err);
		return err;
	}

	return 0;
}

static int set_procedure_state(struct bt_conn *conn, bool enabled)
{
	const struct bt_le_cs_procedure_enable_param params = {
		.config_id = CS_CONFIG_ID,
		.enable = enabled ? BT_CONN_LE_CS_PROCEDURES_ENABLED
				  : BT_CONN_LE_CS_PROCEDURES_DISABLED,
	};

	k_sem_reset(&sem_procedure_enabled);
	int err = bt_le_cs_procedure_enable(conn, &params);

	if (err) {
		LOG_ERR("Cannot %s CS procedure (err %d)", enabled ? "enable" : "disable", err);
		return err;
	}

	err = take_setup_sem(&sem_procedure_enabled, K_SECONDS(5),
			     enabled ? "procedure enable" : "procedure disable");
	if (err) {
		return err;
	}

	return procedure_is_enabled == enabled ? 0 : -EIO;
}

static void report_distance(const struct distance_estimate *estimate)
{
	const char *method;
	float distance;
	uint16_t samples;

	if (estimate->phase_valid) {
		method = "PHASE";
		distance = estimate->phase_m;
		samples = estimate->phase_samples;
	} else if (estimate->rtt_valid) {
		method = "RTT";
		distance = estimate->rtt_m;
		samples = estimate->rtt_samples;
	} else {
		return;
	}

	distance_sequence++;
	printk("DIST_DATA:ANCHOR:%d|SEQ:%u|T_MS:%lld|METHOD:%s|RAW_VAL:%.3f|"
	       "SAMPLES:%u|QUALITY:OK|PBR_VALID:%u|PBR_VAL:%.3f|PBR_SAMPLES:%u|"
	       "PBR_RMSE:%.4f|RTT_VALID:%u|RTT_VAL:%.3f|RTT_SAMPLES:%u|RTT_STD:%.3f|"
	       "RTT_RECORDS:%u|RTT_AA_FAIL:%u|RTT_RSSI_MISS:%u|RTT_TIME_MISS:%u|"
	       "RTT_DIAG_VAL:%.3f\n",
	       ANCHOR_ID, distance_sequence, (long long)k_uptime_get(), method, (double)distance,
	       samples, estimate->phase_valid ? 1u : 0u, (double)estimate->phase_m,
	       estimate->phase_samples, (double)estimate->phase_rmse_rad,
	       estimate->rtt_valid ? 1u : 0u, (double)estimate->rtt_m,
	       estimate->rtt_samples, (double)estimate->rtt_stddev_m,
	       estimate->rtt_records, estimate->rtt_aa_failures,
	       estimate->rtt_rssi_missing, estimate->rtt_timing_missing,
	       (double)estimate->rtt_diagnostic_m);
}

static int perform_measurement_attempt(struct bt_conn *conn)
{
	reset_measurement_state();

	int err = set_procedure_state(conn, true);
	if (err) {
		return err;
	}

	err = k_sem_take(&sem_procedure_done, K_SECONDS(5));
	if (err) {
		LOG_WRN("No complete commanded CS procedure received");
		goto cleanup;
	}

	/* max_procedure_count=1 consumes the enabled procedure when its sole
	 * procedure completes or aborts. No separate "disabled" callback is
	 * generated, so keep the local state in sync with the controller.
	 */
	procedure_is_enabled = false;

	if (!local_procedure_valid || !distance_estimation_in_progress) {
		err = procedure_result_status != -EINPROGRESS ? procedure_result_status : -EIO;
		goto cleanup;
	}

	err = k_sem_take(&sem_rd_ready, K_SECONDS(2));
	if (err) {
		LOG_WRN("Timeout waiting for peer ranging data");
		goto cleanup;
	}

	if (most_recent_peer_ranging_counter != most_recent_local_ranging_counter) {
		LOG_WRN("Ranging counter mismatch (peer %d, local %d)",
			most_recent_peer_ranging_counter, most_recent_local_ranging_counter);
		err = -EIO;
		goto cleanup;
	}

	rd_complete_status = -ETIMEDOUT;
	k_sem_reset(&sem_rd_complete);
	err = bt_ras_rreq_cp_get_ranging_data(conn, &peer_steps,
					      most_recent_peer_ranging_counter,
					      ranging_data_get_complete_cb);
	if (err) {
		LOG_ERR("Cannot request peer ranging data (err %d)", err);
		goto cleanup;
	}

	err = k_sem_take(&sem_rd_complete, K_SECONDS(3));
	if (err || rd_complete_status) {
		LOG_ERR("Peer ranging data unavailable (wait %d, transfer %d)", err,
			rd_complete_status);
		err = err ? err : rd_complete_status;
		goto cleanup;
	}

	struct distance_estimate estimate;

	if (!estimate_distance(&local_steps, &peer_steps, num_antenna_paths,
			       BT_CONN_LE_CS_ROLE_INITIATOR, selected_rtt_type, &estimate)) {
		err = -ERANGE;
		goto cleanup;
	}

	report_distance(&estimate);
	err = 0;

cleanup:
	reset_measurement_buffers();
	k_sem_reset(&sem_rd_ready);

	/* This path is normally used only when no terminal procedure callback was
	 * received. A completed one-shot is already stopped by the controller.
	 */
	if (procedure_is_enabled) {
		int disable_err = set_procedure_state(conn, false);

		if (disable_err && disable_err != -EACCES) {
			LOG_WRN("Commanded CS cleanup failed (err %d)", disable_err);
		}
		procedure_is_enabled = false;
	}

	return err;
}

static int perform_one_measurement(struct bt_conn *conn)
{
	int err = -EIO;

	for (int attempt = 1; attempt <= MEASUREMENT_MAX_ATTEMPTS; attempt++) {
		err = perform_measurement_attempt(conn);
		if (err == 0 || !atomic_get(&connection_active)) {
			return err;
		}

		LOG_WRN("Commanded CS attempt %d/%d failed (err %d)", attempt,
			MEASUREMENT_MAX_ATTEMPTS, err);
		if (attempt < MEASUREMENT_MAX_ATTEMPTS) {
			k_sleep(MEASUREMENT_RETRY_DELAY);
		}
	}

	return err;
}

static void commanded_ranging_loop(struct bt_conn *conn)
{
	atomic_set(&measurement_ready, 1);
	report_cs_status("READY", 0);

	while (atomic_get(&connection_active)) {
		int err = k_sem_take(&sem_measure_request, K_SECONDS(1));

		if (err) {
			continue;
		}

		atomic_set(&measurement_ready, 0);
		atomic_set(&measurement_busy, 1);
		report_cs_status("BUSY", 0);
		err = perform_one_measurement(conn);
		atomic_set(&measurement_busy, 0);

		if (err) {
			report_cs_status("ERROR", err);
			/* perform_one_measurement() has already exhausted the local
			 * retries. Recreate the ACL/CS state instead of repeatedly using
			 * a controller schedule that may remain permanently conflicted.
			 */
			report_cs_status("RECOVERING", 0);
			return;
		}
		if (atomic_get(&connection_active)) {
			atomic_set(&measurement_ready, 1);
			report_cs_status("READY", 0);
		}
	}

	atomic_clear(&measurement_ready);
	atomic_clear(&measurement_busy);
}

int main(void)
{
	int err;

	LOG_INF("Starting CS anchor %d (target %s)", ANCHOR_ID, TARGET_DEVICE_NAME);
	dk_leds_init();
	err = console_init();
	if (err) {
		LOG_ERR("Console input initialization failed (err %d)", err);
		return 0;
	}
	k_sem_give(&sem_console_initialized);
	report_cs_status("BOOT", 0);

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth initialization failed (err %d)", err);
		return 0;
	}

	err = scan_init();
	if (err) {
		return 0;
	}

	while (true) {
		reset_measurement_state();
		k_sem_reset(&sem_connected);
		connection_status = -ETIMEDOUT;

		LOG_INF("Scanning for %s", TARGET_DEVICE_NAME);
		err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
		if (err) {
			LOG_ERR("Cannot start scan (err %d)", err);
			k_sleep(K_SECONDS(1));
			continue;
		}

		err = k_sem_take(&sem_connected, K_SECONDS(30));
		(void)bt_scan_stop();
		if (err || connection_status || connection == NULL) {
			LOG_WRN("CS_TAG connection attempt timed out or failed");
			k_sleep(K_SECONDS(1));
			continue;
		}

		/* Keep a main-thread reference while the callback owns the global reference. */
		struct bt_conn *conn = bt_conn_ref(connection);

		err = configure_connection(conn);
		if (err) {
			LOG_ERR("Connection setup failed (err %d)", err);
		} else {
			commanded_ranging_loop(conn);
		}

		if (atomic_get(&connection_active)) {
			k_sem_reset(&sem_disconnected);
			err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			if (err == 0) {
				(void)k_sem_take(&sem_disconnected, K_SECONDS(5));
			}
		}

		bt_conn_unref(conn);
		reset_measurement_state();
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
