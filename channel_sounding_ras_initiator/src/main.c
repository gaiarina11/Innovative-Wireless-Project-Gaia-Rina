/* Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Channel Sounding Initiator with Ranging Requestor sample
 * Multi-connection (3 reflector) + trilateration
 */

 #include <zephyr/kernel.h>
 #include <zephyr/types.h>
 #include <zephyr/bluetooth/cs.h>
 #include <zephyr/bluetooth/gatt.h>
 #include <zephyr/bluetooth/conn.h>
 #include <bluetooth/scan.h>
 #include <bluetooth/services/ras.h>
 #include <bluetooth/gatt_dm.h>
 #include "distance_estimation.h"
 #include <dk_buttons_and_leds.h>
 #include <zephyr/logging/log.h>
 LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);
 #define CON_STATUS_LED DK_LED1
 #define CS_CONFIG_ID 0
 #define NUM_MODE_0_STEPS 3
 #define PROCEDURE_COUNTER_NONE (-1)
 #define LOCAL_PROCEDURE_MEM                                                     \
 ((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) + \
 (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))
 /* Semafori globali per la fase di configurazione sequenziale */
 static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
 static K_SEM_DEFINE(sem_config_created, 0, 1);
 static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
 static K_SEM_DEFINE(sem_connected, 0, 1);
 static K_SEM_DEFINE(sem_discovery_done, 0, 1);
 static K_SEM_DEFINE(sem_mtu_exchange_done, 0, 1);
 static K_SEM_DEFINE(sem_security, 0, 1);
 /* Gestione Multiconnessione (3 Reflector) */
 #define MAX_REFLECTORS 3
 static struct bt_conn *connections[MAX_REFLECTORS] = {NULL};
 static uint8_t connected_count = 0;
 /* Capabilities salvate per ogni reflector */
 static struct bt_conn_le_cs_capabilities remote_caps[MAX_REFLECTORS];
 
 static K_SEM_DEFINE(sem_disconnected, 0, 1);
 static uint8_t target_reflector_idx = 0; /* Traccia quale reflector stiamo gestendo */
 
 /* Buffer separati per ogni reflector */
 NET_BUF_SIMPLE_DEFINE_STATIC(local_steps_0, LOCAL_PROCEDURE_MEM);
 NET_BUF_SIMPLE_DEFINE_STATIC(peer_steps_0, BT_RAS_PROCEDURE_MEM);
 NET_BUF_SIMPLE_DEFINE_STATIC(local_steps_1, LOCAL_PROCEDURE_MEM);
 NET_BUF_SIMPLE_DEFINE_STATIC(peer_steps_1, BT_RAS_PROCEDURE_MEM);
 NET_BUF_SIMPLE_DEFINE_STATIC(local_steps_2, LOCAL_PROCEDURE_MEM);
 NET_BUF_SIMPLE_DEFINE_STATIC(peer_steps_2, BT_RAS_PROCEDURE_MEM);
 static struct net_buf_simple *local_steps_ptrs[MAX_REFLECTORS] = {
 &local_steps_0, &local_steps_1, &local_steps_2};
 static struct net_buf_simple *peer_steps_ptrs[MAX_REFLECTORS] = {
 &peer_steps_0, &peer_steps_1, &peer_steps_2};
 /* Variabili di stato per ogni reflector */
 static int32_t most_recent_peer_ranging_counter[MAX_REFLECTORS] = {
 PROCEDURE_COUNTER_NONE, PROCEDURE_COUNTER_NONE, PROCEDURE_COUNTER_NONE};
 static int32_t most_recent_local_ranging_counter[MAX_REFLECTORS] = {
 PROCEDURE_COUNTER_NONE, PROCEDURE_COUNTER_NONE, PROCEDURE_COUNTER_NONE};
 static int32_t dropped_ranging_counter[MAX_REFLECTORS] = {
 PROCEDURE_COUNTER_NONE, PROCEDURE_COUNTER_NONE, PROCEDURE_COUNTER_NONE};
 static bool distance_estimation_in_progress[MAX_REFLECTORS] = {false, false, false};
 static uint8_t n_ap_arr[MAX_REFLECTORS];
 /* Semafori di ranging per ogni connessione */
 static struct k_sem sem_procedure_done_arr[MAX_REFLECTORS];
 static struct k_sem sem_rd_ready_arr[MAX_REFLECTORS];
 static struct k_sem sem_rd_complete_arr[MAX_REFLECTORS];
 static int get_reflector_index(struct bt_conn *conn)
 {
 for (int i = 0; i < MAX_REFLECTORS; i++)
 {
 if (connections[i] == conn)
 {
 return i;
 }
 }
 return -1;
 }
 /* ── Callbacks CS ─────────────────────────────────────────────────────────── */
 static void subevent_result_cb(struct bt_conn *conn,
   struct bt_conn_le_cs_subevent_result *result)
 {
 int idx = get_reflector_index(conn);
 if (idx < 0)
 return;
 
 LOG_INF("[R%d] Subevent result %d", idx, result->header.procedure_counter);
 
 struct net_buf_simple *local_steps = local_steps_ptrs[idx];
 
 if (distance_estimation_in_progress[idx])
 {
 LOG_WRN("[R%d] Estimation in progress, dropping procedure.", idx);
 dropped_ranging_counter[idx] = result->header.procedure_counter;
 return;
 }
 
 /* FIX 1: Non eseguire il return se il subevent fallisce! */
 if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED)
 {
 LOG_WRN("[R%d] Subevent aborted", idx);
 dropped_ranging_counter[idx] = result->header.procedure_counter;
 net_buf_simple_reset(local_steps);
 }
 else
 {
 /* Se non è abortito, estrai i dati nei buffer */
 if (dropped_ranging_counter[idx] != result->header.procedure_counter)
 {
 if (result->step_data_buf)
 {
 if (result->step_data_buf->len <= net_buf_simple_tailroom(local_steps))
 {
 uint16_t len = result->step_data_buf->len;
 uint8_t *step_data = net_buf_simple_pull_mem(
 result->step_data_buf, len);
 net_buf_simple_add_mem(local_steps, step_data, len);
 }
 else
 {
 LOG_ERR("[R%d] Not enough memory for step data.", idx);
 net_buf_simple_reset(local_steps);
 dropped_ranging_counter[idx] = result->header.procedure_counter;
 }
 }
 }
 }
 
 /* FIX 2: Controlla sempre lo stato di fine procedura per sbloccare il semaforo */
 if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE)
 {
 dropped_ranging_counter[idx] = PROCEDURE_COUNTER_NONE;
 n_ap_arr[idx] = result->header.num_antenna_paths;
 most_recent_local_ranging_counter[idx] = result->header.procedure_counter;
 distance_estimation_in_progress[idx] = true;
 k_sem_give(&sem_procedure_done_arr[idx]);
 }
 else if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED)
 {
 LOG_WRN("[R%d] Procedure aborted", idx);
 net_buf_simple_reset(local_steps);
 dropped_ranging_counter[idx] = PROCEDURE_COUNTER_NONE;
 distance_estimation_in_progress[idx] = false; /* Segnala al main il fallimento */
 k_sem_give(&sem_procedure_done_arr[idx]);  /* SBLOCCA IL MAIN! */
 }
 }
 static void ranging_data_get_complete_cb(struct bt_conn *conn,
 uint16_t ranging_counter, int err)
 {
 int idx = get_reflector_index(conn);
 if (idx < 0)
 return;
 if (err)
 {
 LOG_ERR("[R%d] Error getting ranging data (err %d)", idx, err);
 return;
 }
 LOG_INF("[R%d] Ranging data complete, counter %d", idx, ranging_counter);
 k_sem_give(&sem_rd_complete_arr[idx]);
 }
 static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
 {
 int idx = get_reflector_index(conn);
 if (idx < 0)
 return;
 LOG_INF("[R%d] Ranging data ready %d", idx, ranging_counter);
 most_recent_peer_ranging_counter[idx] = ranging_counter;
 k_sem_give(&sem_rd_ready_arr[idx]);
 }
 static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
 {
 int idx = get_reflector_index(conn);
 if (idx < 0)
 return;
 LOG_INF("[R%d] Ranging data overwritten %d", idx, ranging_counter);
 }
 static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
 struct bt_gatt_exchange_params *params)
 {
 if (err)
 {
 LOG_ERR("MTU exchange failed (err %d)", err);
 return;
 }
 LOG_INF("MTU exchange success (%u)", bt_gatt_get_mtu(conn));
 k_sem_give(&sem_mtu_exchange_done);
 }
 static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
 {
 int err;
 LOG_INF("Discovery succeeded");
 struct bt_conn *conn = bt_gatt_dm_conn_get(dm);
 bt_gatt_dm_data_print(dm);
 err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);
 if (err)
 {
 LOG_ERR("RAS RREQ alloc failed (err %d)", err);
 }
 err = bt_gatt_dm_data_release(dm);
 if (err)
 {
 LOG_ERR("Discovery data release failed (err %d)", err);
 }
 k_sem_give(&sem_discovery_done);
 }
 static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
 {
 LOG_INF("Service not found, disconnecting");
 bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
 }
 static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
 {
 LOG_INF("Discovery error (err %d)", err);
 bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
 }
 static struct bt_gatt_dm_cb discovery_cb = {
 .completed = discovery_completed_cb,
 .service_not_found = discovery_service_not_found_cb,
 .error_found = discovery_error_found_cb,
 };
 static void security_changed(struct bt_conn *conn, bt_security_t level,
 enum bt_security_err err)
 {
 char addr[BT_ADDR_LE_STR_LEN];
 bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
 if (err)
 {
 LOG_ERR("Security failed: %s level %u err %d %s",
 addr, level, err, bt_security_err_to_str(err));
 return;
 }
 LOG_INF("Security changed: %s level %u", addr, level);
 k_sem_give(&sem_security);
 }
 static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
 {
 return false;
 }
 static void connected_cb(struct bt_conn *conn, uint8_t err)
 {
 char addr[BT_ADDR_LE_STR_LEN];
 (void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
 LOG_INF("Connected to %s (err 0x%02X)", addr, err);
 
 if (err)
 {
 bt_conn_unref(conn);
 return;
 }
 
 /* Salva la connessione direttamente nella cella del reflector attivo */
 connections[target_reflector_idx] = bt_conn_ref(conn);
 k_sem_give(&sem_connected);
 dk_set_led_on(CON_STATUS_LED);
 }
 
 static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
 {
 LOG_INF("Disconnected (reason 0x%02X)", reason);
 if (connections[target_reflector_idx] == conn)
 {
 bt_conn_unref(conn);
 connections[target_reflector_idx] = NULL;
 }
 dk_set_led_off(CON_STATUS_LED);
 k_sem_give(&sem_disconnected); /* Sblocca il main dopo la disconnessione */
 }
 
 static void remote_capabilities_cb(struct bt_conn *conn,
   struct bt_conn_le_cs_capabilities *params)
 {
 int idx = get_reflector_index(conn);
 if (idx >= 0 && params)
 {
 remote_caps[idx] = *params;
 LOG_INF("[R%d] Capabilities: antennas=%d snr=%d cs_sync_2m=%d",
 idx,
 params->num_antennas_supported,
 params->tx_snr_capability,
 params->cs_sync_2m_phy_supported);
 }
 LOG_INF("CS capability exchange completed.");
 k_sem_give(&sem_remote_capabilities_obtained);
 }
 static void config_created_cb(struct bt_conn *conn, struct bt_conn_le_cs_config *config)
 {
 LOG_INF("CS config created. ID: %d", config->id);
 k_sem_give(&sem_config_created);
 }
 static void security_enabled_cb(struct bt_conn *conn)
 {
 LOG_INF("CS security enabled.");
 k_sem_give(&sem_cs_security_enabled);
 }
 static void procedure_enabled_cb(struct bt_conn *conn,
 struct bt_conn_le_cs_procedure_enable_complete *params)
 {
 LOG_INF("CS procedures %s.", params->state == 1 ? "enabled" : "disabled");
 }
 /* ── Scan ─────────────────────────────────────────────────────────────────── */
 static void scan_filter_match(struct bt_scan_device_info *device_info,
  struct bt_scan_filter_match *filter_match,
  bool connectable)
 {
 char addr[BT_ADDR_LE_STR_LEN];
 bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
 LOG_INF("Filter matched: %s connectable: %d", addr, connectable);
 }
 static void scan_connecting_error(struct bt_scan_device_info *device_info)
 {
 int err;
 LOG_INF("Connecting failed, restarting scan");
 err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
 if (err)
 {
 LOG_ERR("Failed to restart scan (err %i)", err);
 }
 }
 static void scan_connecting(struct bt_scan_device_info *device_info,
 struct bt_conn *conn)
 {
 LOG_INF("Connecting...");
 }
 BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
 scan_connecting_error, scan_connecting);
 static int scan_init(void)
 {
 int err;
 struct bt_scan_init_param param = {
 .scan_param = NULL,
 .conn_param = BT_LE_CONN_PARAM(40, 40, 0, 400), /* 200 ms */
 .connect_if_match = 1};
 bt_scan_init(&param);
 bt_scan_cb_register(&scan_cb);
 err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_RANGING_SERVICE);
 if (err)
 {
 LOG_ERR("Cannot set scan filter (err %d)", err);
 return err;
 }
 err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
 if (err)
 {
 LOG_ERR("Cannot enable filter (err %d)", err);
 return err;
 }
 return 0;
 }
 BT_CONN_CB_DEFINE(conn_cb) = {
 .connected = connected_cb,
 .disconnected = disconnected_cb,
 .le_param_req = le_param_req,
 .security_changed = security_changed,
 .le_cs_remote_capabilities_available = remote_capabilities_cb,
 .le_cs_config_created = config_created_cb,
 .le_cs_security_enabled = security_enabled_cb,
 .le_cs_procedure_enabled = procedure_enabled_cb,
 .le_cs_subevent_data_available = subevent_result_cb,
 };
 /* ── main ─────────────────────────────────────────────────────────────────── */
 int main(void)
 {
 int err;
 
 LOG_INF("CS Initiator — Multi-Connection Pure Sequential");
 
 dk_leds_init();
 
 err = bt_enable(NULL);
 if (err)
 {
 LOG_ERR("Bluetooth init failed (err %d)", err);
 return 0;
 }
 
 bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
 printk("\n[RESET] Vecchi accoppiamenti cancellati.\n\n");
 
 err = scan_init();
 if (err)
 {
 LOG_ERR("Scan init failed (err %d)", err);
 return 0;
 }
 
 memset(remote_caps, 0, sizeof(remote_caps));
 
 for (int i = 0; i < MAX_REFLECTORS; i++)
 {
 k_sem_init(&sem_procedure_done_arr[i], 0, 1);
 k_sem_init(&sem_rd_ready_arr[i], 0, 1);
 k_sem_init(&sem_rd_complete_arr[i], 0, 1);
 }
 
 /* Ciclo infinito Sequenziale Puro */
 while (true)
 {
 
 for (int i = 0; i < MAX_REFLECTORS; i++)
 {
 target_reflector_idx = i; /* Imposta l'indice per le callback */
 
 LOG_INF("====== [R%d] Fase 1: Avvio scansione e Connessione ======", i);
 
 err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
 if (err)
 {
 LOG_ERR("Scan start failed (err %i)", err);
 k_sleep(K_MSEC(1000));
 continue;
 }
 
 k_sem_take(&sem_connected, K_FOREVER);
 bt_scan_stop();
 
 struct bt_conn *current_conn = connections[i];
 if (!current_conn)
 {
 continue;
 }
 
 LOG_INF("====== [R%d] Fase 2: Configurazione Servizi e Parametri ======", i);
 
 err = bt_conn_set_security(current_conn, BT_SECURITY_L2);
 if (err)
 {
 LOG_ERR("Security failed");
 goto cleanup_conn;
 }
 k_sem_take(&sem_security, K_FOREVER);
 
 static struct bt_gatt_exchange_params mtu_params;
 mtu_params.func = mtu_exchange_cb;
 bt_gatt_exchange_mtu(current_conn, &mtu_params);
 k_sem_take(&sem_mtu_exchange_done, K_FOREVER);
 
 err = bt_gatt_dm_start(current_conn, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
 if (err)
 {
 LOG_ERR("Discovery failed");
 goto cleanup_conn;
 }
 k_sem_take(&sem_discovery_done, K_FOREVER);
 
 const struct bt_le_cs_set_default_settings_param default_settings = {
 .enable_initiator_role = true,
 .enable_reflector_role = false,
 .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
 .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
 };
 bt_le_cs_set_default_settings(current_conn, &default_settings);
 
 bt_ras_rreq_rd_overwritten_subscribe(current_conn, ranging_data_overwritten_cb);
 bt_ras_rreq_rd_ready_subscribe(current_conn, ranging_data_ready_cb);
 bt_ras_rreq_on_demand_rd_subscribe(current_conn);
 bt_ras_rreq_cp_subscribe(current_conn);
 
 err = bt_le_cs_read_remote_supported_capabilities(current_conn);
 k_sem_take(&sem_remote_capabilities_obtained, K_FOREVER);
 
 struct bt_le_cs_create_config_params config_params = {
 .id = CS_CONFIG_ID,
 .main_mode_type = BT_CONN_LE_CS_MAIN_MODE_2,
 .sub_mode_type = BT_CONN_LE_CS_SUB_MODE_UNUSED,
 .min_main_mode_steps = 10,
 .max_main_mode_steps = 20,
 .mode_0_steps = NUM_MODE_0_STEPS,
 .role = BT_CONN_LE_CS_ROLE_INITIATOR,
 .rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY,
 .cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY,
 .channel_map_repetition = 5,
 .channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
 .ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT,
 .ch3c_jump = 2,
 };
 bt_le_cs_set_valid_chmap_bits(config_params.channel_map);
 
 err = bt_le_cs_create_config(current_conn, &config_params, BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
 k_sem_take(&sem_config_created, K_FOREVER);
 
 err = bt_le_cs_security_enable(current_conn);
 k_sem_take(&sem_cs_security_enabled, K_FOREVER);
 
 uint8_t antenna_cfg = (remote_caps[i].num_antennas_supported >= 2) ? BT_LE_CS_TONE_ANTENNA_CONFIGURATION_INDEX_ONE : 0;
 uint8_t proc_phy = remote_caps[i].cs_sync_2m_phy_supported ? (uint8_t)BT_LE_CS_PROCEDURE_PHY_2M : (uint8_t)BT_LE_CS_PROCEDURE_PHY_1M;
 
 const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
 .config_id = CS_CONFIG_ID,
 .max_procedure_len = 12,
 .min_procedure_interval = 16,
 .max_procedure_interval = 160,
 .max_procedure_count = 1, /* Esegui una sola misura */
 .min_subevent_len = 5000,
 .max_subevent_len = 5000,
 .tone_antenna_config_selection = antenna_cfg,
 .phy = proc_phy,
 .tx_power_delta = 0x80,
 .preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
 .snr_control_initiator = 0xFF,
 .snr_control_reflector = 0xFF,
 };
 bt_le_cs_set_procedure_parameters(current_conn, &procedure_params);
 
 LOG_INF("====== [R%d] Fase 3: Avvio Ranging ed Estrazione Dati ======", i);
 
 struct bt_le_cs_procedure_enable_param enable_params = {
 .config_id = CS_CONFIG_ID,
 .enable = 1,
 };
 
 err = bt_le_cs_procedure_enable(current_conn, &enable_params);
 if (err)
 {
 goto cleanup_conn;
 }
 
 err = k_sem_take(&sem_procedure_done_arr[i], K_SECONDS(2));
 if (err != 0 || !distance_estimation_in_progress[i])
 {
 goto reset_buffers;
 }
 
 err = k_sem_take(&sem_rd_ready_arr[i], K_SECONDS(1));
 if (err)
 {
 goto reset_buffers;
 }
 
 if (most_recent_peer_ranging_counter[i] != most_recent_local_ranging_counter[i])
 {
 goto reset_buffers;
 }
 
 err = bt_ras_rreq_cp_get_ranging_data(current_conn, peer_steps_ptrs[i], most_recent_peer_ranging_counter[i], ranging_data_get_complete_cb);
 if (err)
 {
 goto reset_buffers;
 }
 
 err = k_sem_take(&sem_rd_complete_arr[i], K_SECONDS(3));
 if (err)
 {
 goto reset_buffers;
 }
 
 /* Elaborazione e stampa */
 {
 char addr_str[BT_ADDR_LE_STR_LEN];
 bt_addr_le_to_str(bt_conn_get_dst(current_conn), addr_str, sizeof(addr_str));
 printk("CS_DATA:REFLECTOR_ID:%d|MAC:%s\n", i, addr_str);
 estimate_distance(local_steps_ptrs[i], peer_steps_ptrs[i], n_ap_arr[i], BT_CONN_LE_CS_ROLE_INITIATOR);
 }
 
 reset_buffers:
 net_buf_simple_reset(local_steps_ptrs[i]);
 net_buf_simple_reset(peer_steps_ptrs[i]);
 distance_estimation_in_progress[i] = false;
 
 cleanup_conn:
 LOG_INF("====== [R%d] Fase 4: Disconnessione Forzata Radio ======", i);
 if (connections[i])
 {
 err = bt_conn_disconnect(connections[i], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
 if (!err)
 {
 k_sem_take(&sem_disconnected, K_FOREVER); /* Aspetta la rimozione reale della connessione */
 }
 }
 
 /* Piccola pausa prima di scansionare il prossimo reflector */
 k_sleep(K_MSEC(100));
 
 } /* Fine ciclo for dei 3 reflector */
 } /* Fine del while(true) */
 
 return 0;
 }