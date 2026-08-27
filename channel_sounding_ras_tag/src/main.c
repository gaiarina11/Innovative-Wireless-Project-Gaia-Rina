/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Multi-link Channel Sounding Reflector with Ranging Responder.
 */

#include <errno.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <bluetooth/services/ras.h>
#include <dk_buttons_and_leds.h>

LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#define CON_STATUS_LED DK_LED1
#define MAX_ANCHOR_CONNECTIONS 3
#define ADV_RETRY_DELAY K_MSEC(500)

static struct bt_conn *connections[MAX_ANCHOR_CONNECTIONS];
static atomic_t connected_count;
static atomic_t advertising_active;

K_MSGQ_DEFINE(connection_config_queue, sizeof(struct bt_conn *), MAX_ANCHOR_CONNECTIONS,
	      sizeof(void *));

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_RANGING_SERVICE_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void advertising_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(advertising_work, advertising_work_handler);

static int find_connection_slot(struct bt_conn *conn)
{
	for (int i = 0; i < ARRAY_SIZE(connections); i++) {
		if (connections[i] == conn) {
			return i;
		}
	}

	return -1;
}

static int find_free_slot(void)
{
	for (int i = 0; i < ARRAY_SIZE(connections); i++) {
		if (connections[i] == NULL) {
			return i;
		}
	}

	return -1;
}

static void request_advertising_after(k_timeout_t delay)
{
	if (atomic_get(&connected_count) < MAX_ANCHOR_CONNECTIONS &&
	    !atomic_get(&advertising_active)) {
		(void)k_work_reschedule(&advertising_work, delay);
	}
}

static void request_advertising(void)
{
	request_advertising_after(K_NO_WAIT);
}

static void advertising_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (atomic_get(&connected_count) >= MAX_ANCHOR_CONNECTIONS) {
		return;
	}

	int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err == 0) {
		atomic_set(&advertising_active, 1);
		LOG_INF("Advertising as %s (%ld/%d anchors connected)", CONFIG_BT_DEVICE_NAME,
			(long)atomic_get(&connected_count), MAX_ANCHOR_CONNECTIONS);
	} else if (err == -EALREADY) {
		/* The connected callback can run just before legacy advertising is
		 * fully stopped by the host. Retry instead of losing the request for
		 * the next anchor.
		 */
		LOG_INF("Advertising is still stopping; retrying");
		(void)k_work_reschedule(&advertising_work, ADV_RETRY_DELAY);
	} else {
		LOG_WRN("Advertising start failed (err %d), retrying", err);
		(void)k_work_reschedule(&advertising_work, ADV_RETRY_DELAY);
	}
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (err) {
		atomic_clear(&advertising_active);
		LOG_ERR("Incoming connection from %s failed (err 0x%02x)", addr, err);
		request_advertising_after(ADV_RETRY_DELAY);
		return;
	}

	/* Legacy connectable advertising terminates after a connection. The
	 * delayed restart avoids racing the host's advertising-stop transition.
	 */
	atomic_clear(&advertising_active);

	int slot = find_free_slot();

	if (slot < 0) {
		LOG_ERR("No free slot for connection from %s", addr);
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_LOW_RESOURCES);
		return;
	}

	connections[slot] = bt_conn_ref(conn);
	atomic_inc(&connected_count);
	dk_set_led_on(CON_STATUS_LED);
	LOG_INF("Anchor connected in slot %d: %s (%ld/%d)", slot, addr,
		(long)atomic_get(&connected_count), MAX_ANCHOR_CONNECTIONS);

	/* The main thread receives its own reference so a fast disconnect cannot
	 * invalidate the object before the CS settings are applied.
	 */
	struct bt_conn *queued_conn = bt_conn_ref(conn);

	if (k_msgq_put(&connection_config_queue, &queued_conn, K_NO_WAIT) != 0) {
		LOG_ERR("Connection configuration queue full");
		bt_conn_unref(queued_conn);
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_LOW_RESOURCES);
	}

	request_advertising_after(ADV_RETRY_DELAY);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	int slot = find_connection_slot(conn);

	LOG_WRN("Anchor disconnected from slot %d (reason 0x%02x)", slot, reason);
	if (slot >= 0) {
		bt_conn_unref(connections[slot]);
		connections[slot] = NULL;
		atomic_dec(&connected_count);
	}

	if (atomic_get(&connected_count) == 0) {
		dk_set_led_off(CON_STATUS_LED);
	}

	request_advertising();
}

static void remote_capabilities_cb(struct bt_conn *conn,
				   struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(params);

	LOG_INF("CS capability exchange completed for connection %u", bt_conn_index(conn));
}

static void config_created_cb(struct bt_conn *conn, struct bt_conn_le_cs_config *config)
{
	LOG_INF("CS configuration %u created for connection %u", config->id,
		bt_conn_index(conn));
}

static void security_enabled_cb(struct bt_conn *conn)
{
	LOG_INF("CS security enabled for connection %u", bt_conn_index(conn));
}

static void procedure_enabled_cb(struct bt_conn *conn,
				 struct bt_conn_le_cs_procedure_enable_complete *params)
{
	LOG_INF("CS procedures %s for connection %u",
		params->state == BT_CONN_LE_CS_PROCEDURES_ENABLED ? "enabled" : "disabled",
		bt_conn_index(conn));
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_cs_remote_capabilities_available = remote_capabilities_cb,
	.le_cs_config_created = config_created_cb,
	.le_cs_security_enabled = security_enabled_cb,
	.le_cs_procedure_enabled = procedure_enabled_cb,
};

static int configure_reflector_connection(struct bt_conn *conn)
{
	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = false,
		.enable_reflector_role = true,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	int err = bt_le_cs_set_default_settings(conn, &default_settings);

	if (err) {
		LOG_ERR("Cannot configure reflector role for connection %u (err %d)",
			bt_conn_index(conn), err);
		return err;
	}

	LOG_INF("Reflector role configured for connection %u", bt_conn_index(conn));
	return 0;
}

int main(void)
{
	LOG_INF("Starting multi-link CS Tag (%d anchor slots)", MAX_ANCHOR_CONNECTIONS);
	dk_leds_init();

	int err = bt_enable(NULL);

	if (err) {
		LOG_ERR("Bluetooth initialization failed (err %d)", err);
		return 0;
	}

	request_advertising();

	while (true) {
		struct bt_conn *conn;

		if (k_msgq_get(&connection_config_queue, &conn, K_FOREVER) == 0) {
			(void)configure_reflector_connection(conn);
			bt_conn_unref(conn);
		}
	}

	return 0;
}
