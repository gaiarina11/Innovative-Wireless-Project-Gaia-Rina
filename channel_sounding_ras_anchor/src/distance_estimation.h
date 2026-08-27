/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/bluetooth/cs.h>

struct distance_estimate {
	bool valid;
	bool phase_valid;
	bool rtt_valid;
	float phase_m;
	float phase_rmse_rad;
	float rtt_m;
	float rtt_stddev_m;
	float rtt_diagnostic_m;
	uint16_t phase_samples;
	uint16_t rtt_samples;
	uint16_t rtt_records;
	uint16_t rtt_aa_failures;
	uint16_t rtt_rssi_missing;
	uint16_t rtt_timing_missing;
};

bool estimate_distance(struct net_buf_simple *local_steps, struct net_buf_simple *peer_steps,
		       uint8_t n_ap, enum bt_conn_le_cs_role role,
		       enum bt_conn_le_cs_rtt_type rtt_type,
		       struct distance_estimate *estimate);
