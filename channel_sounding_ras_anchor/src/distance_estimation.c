/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding distance estimation for Ranging Requestor
 */

#include "distance_estimation.h"

#include "zephyr/bluetooth/hci_types.h"
#include <float.h>
#include <math.h>
#include <zephyr/bluetooth/cs.h>
#include <bluetooth/services/ras.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

#define CS_FREQUENCY_MHZ(ch)	(2402u + 1u * (ch))
#define CS_FREQUENCY_HZ(ch)	(CS_FREQUENCY_MHZ(ch) * 1000000.0f)
#define SPEED_OF_LIGHT_M_PER_S	(299792458.0f)
#define SPEED_OF_LIGHT_NM_PER_S (SPEED_OF_LIGHT_M_PER_S / 1000000000.0f)
#define PI			3.14159265358979323846f
#define MAX_NUM_RTT_SAMPLES		256
#define MAX_NUM_IQ_SAMPLES		256 * CONFIG_BT_RAS_MAX_ANTENNA_PATHS
#define MIN_PHASE_SAMPLES		20
#define MIN_RTT_SAMPLES			5
#define MAX_PHASE_FIT_RMSE_RAD		0.75f
#define PHASE_OUTLIER_RESIDUAL_RAD	1.25f
#define RTT_OUTLIER_FLOOR_M		0.30f
#define RTT_MAD_SCALE			1.4826f
#define RTT_OUTLIER_SIGMA		3.0f
#define MIN_VALID_DISTANCE_M		0.05f
/* This is a raw-estimate guard, applied before per-anchor calibration. Keep
 * enough headroom for systematic scale/offset errors while rejecting the
 * multi-metre phase failures observed during bring-up.
 */
#define MAX_VALID_DISTANCE_M		3.00f
#define MAX_VALID_RTT_DISTANCE_M	6.00f

struct iq_sample_and_channel {
	bool failed;
	uint8_t channel;
	uint8_t antenna_permutation;
	struct bt_le_cs_iq_sample local_iq_sample;
	struct bt_le_cs_iq_sample peer_iq_sample;
};

struct rtt_timing {
	bool failed;
	int16_t toa_tod_initiator;
	int16_t tod_toa_reflector;
};

static struct iq_sample_and_channel iq_sample_channel_data[MAX_NUM_IQ_SAMPLES];
static struct rtt_timing rtt_timing_data[MAX_NUM_RTT_SAMPLES];
static float rtt_distance_samples[MAX_NUM_RTT_SAMPLES];
static float rtt_sort_scratch[MAX_NUM_RTT_SAMPLES];

struct processing_context {
	uint16_t rtt_timing_data_index;
	uint16_t iq_sample_channel_data_index;
	uint8_t n_ap;
	enum bt_conn_le_cs_role role;
	enum bt_conn_le_cs_rtt_type rtt_type;
	uint16_t rtt_aa_failures;
	uint16_t rtt_rssi_missing;
	uint16_t rtt_timing_missing;
};

static void calc_complex_product(int32_t z_a_real, int32_t z_a_imag, int32_t z_b_real,
				 int32_t z_b_imag, int32_t *z_out_real, int32_t *z_out_imag)
{
	*z_out_real = z_a_real * z_b_real - z_a_imag * z_b_imag;
	*z_out_imag = z_a_real * z_b_imag + z_a_imag * z_b_real;
}

static bool linear_regression(const float *x_values, const float *y_values, uint16_t n_samples,
			      float *slope, float *intercept, float *rmse)
{
	if (n_samples < 2 || slope == NULL || intercept == NULL || rmse == NULL) {
		return false;
	}

	/* Estimates b in y = a + b x */

	float y_mean = 0.0;
	float x_mean = 0.0;

	for (uint16_t i = 0; i < n_samples; i++) {
		y_mean += (y_values[i] - y_mean) / (i + 1);
		x_mean += (x_values[i] - x_mean) / (i + 1);
	}

	float b_est_upper = 0.0;
	float b_est_lower = 0.0;

	for (uint16_t i = 0; i < n_samples; i++) {
		b_est_upper += (x_values[i] - x_mean) * (y_values[i] - y_mean);
		b_est_lower += (x_values[i] - x_mean) * (x_values[i] - x_mean);
	}

	if (fabsf(b_est_lower) <= FLT_EPSILON) {
		return false;
	}

	*slope = b_est_upper / b_est_lower;
	*intercept = y_mean - (*slope * x_mean);

	float residual_sum_squares = 0.0f;

	for (uint16_t i = 0; i < n_samples; i++) {
		float residual = y_values[i] - (*intercept + (*slope * x_values[i]));

		residual_sum_squares += residual * residual;
	}

	*rmse = sqrtf(residual_sum_squares / n_samples);
	return isfinite(*slope) && isfinite(*intercept) && isfinite(*rmse);
}

static void bubblesort_2(float *array1, float *array2, uint16_t len)
{
	bool swapped;
	float temp;

	for (uint16_t i = 0; i < len - 1; i++) {
		swapped = false;
		for (uint16_t j = 0; j < len - i - 1; j++) {
			if (array1[j] > array1[j + 1]) {
				temp = array1[j];
				array1[j] = array1[j + 1];
				array1[j + 1] = temp;
				temp = array2[j];
				array2[j] = array2[j + 1];
				array2[j + 1] = temp;
				swapped = true;
			}
		}

		if (!swapped) {
			break;
		}
	}
}

static void insertion_sort(float *values, uint16_t len)
{
	for (uint16_t i = 1; i < len; i++) {
		float value = values[i];
		uint16_t j = i;

		while (j > 0 && values[j - 1] > value) {
			values[j] = values[j - 1];
			j--;
		}
		values[j] = value;
	}
}

static float median_copy(const float *values, uint16_t len)
{
	memcpy(rtt_sort_scratch, values, len * sizeof(values[0]));
	insertion_sort(rtt_sort_scratch, len);

	if ((len & 1u) != 0u) {
		return rtt_sort_scratch[len / 2];
	}
	return (rtt_sort_scratch[(len / 2) - 1] + rtt_sort_scratch[len / 2]) / 2.0f;
}

static bool estimate_distance_using_phase_slope(struct iq_sample_and_channel *data, uint16_t len,
					float *distance_m, float *fit_rmse_rad,
					uint16_t *valid_samples)
{
	int32_t combined_i;
	int32_t combined_q;
	uint16_t num_angles = 0;
	static float theta[MAX_NUM_IQ_SAMPLES];
	static float frequencies[MAX_NUM_IQ_SAMPLES];
	static float inlier_theta[MAX_NUM_IQ_SAMPLES];
	static float inlier_frequencies[MAX_NUM_IQ_SAMPLES];

	for (uint16_t i = 0; i < len; i++) {
		if (!data[i].failed) {
			calc_complex_product(data[i].local_iq_sample.i, data[i].local_iq_sample.q,
					     data[i].peer_iq_sample.i, data[i].peer_iq_sample.q,
					     &combined_i, &combined_q);

			theta[num_angles] = atan2f((float)combined_q, (float)combined_i);
			frequencies[num_angles] = (float)CS_FREQUENCY_MHZ(data[i].channel);
			num_angles++;
		}
	}

	*valid_samples = num_angles;
	if (num_angles < MIN_PHASE_SAMPLES) {
		return false;
	}

	/* Sort phases by tone frequency */
	bubblesort_2(frequencies, theta, num_angles);

	/* One-dimensional phase unwrapping */
	for (uint16_t i = 1; i < num_angles; i++) {
		float difference = theta[i] - theta[i - 1];

		if (difference > PI) {
			for (uint16_t j = i; j < num_angles; j++) {
				theta[j] -= 2.0f * PI;
			}
		} else if (difference < -PI) {
			for (uint16_t j = i; j < num_angles; j++) {
				theta[j] += 2.0f * PI;
			}
		}
	}

	float phase_slope;
	float phase_intercept;
	float phase_rmse;

	if (!linear_regression(frequencies, theta, num_angles, &phase_slope, &phase_intercept,
			       &phase_rmse)) {
		return false;
	}

	/* Refit once without tones that are inconsistent with the initial phase
	 * line. This prevents a few corrupted PCT values from rotating the slope.
	 */
	uint16_t num_inliers = 0;

	for (uint16_t i = 0; i < num_angles; i++) {
		float residual = theta[i] - (phase_intercept + (phase_slope * frequencies[i]));

		if (fabsf(residual) <= PHASE_OUTLIER_RESIDUAL_RAD) {
			inlier_frequencies[num_inliers] = frequencies[i];
			inlier_theta[num_inliers] = theta[i];
			num_inliers++;
		}
	}

	if (num_inliers < MIN_PHASE_SAMPLES ||
	    !linear_regression(inlier_frequencies, inlier_theta, num_inliers, &phase_slope,
			       &phase_intercept, &phase_rmse)) {
		return false;
	}

	float distance = -phase_slope * (SPEED_OF_LIGHT_M_PER_S / (4 * PI));
	float result_m = distance / 1000000.0f; /* Scale to meters. */

	*distance_m = result_m;
	*fit_rmse_rad = phase_rmse;
	*valid_samples = num_inliers;

	if (!isfinite(result_m) || result_m < MIN_VALID_DISTANCE_M ||
	    result_m > MAX_VALID_DISTANCE_M || phase_rmse > MAX_PHASE_FIT_RMSE_RAD) {
		return false;
	}

	return true;
}

static bool estimate_distance_using_time_of_flight(uint16_t n_samples, float *distance_m,
					    float *stddev_m, uint16_t *valid_samples)
{
	float tof;
	uint16_t candidate_count = 0;

	*distance_m = 0.0f;
	*stddev_m = 0.0f;
	*valid_samples = 0;

	/* Convert all Controller-qualified RTT timings to metres first. */
	for (uint16_t i = 0; i < n_samples; i++) {
		if (!rtt_timing_data[i].failed) {
			tof = (rtt_timing_data[i].toa_tod_initiator -
			       rtt_timing_data[i].tod_toa_reflector) /
			      2;
			/* The controller timing convention can reverse with the local/peer
			 * ordering used by the RAS parser. Propagation distance is unsigned,
			 * so normalize each qualified observation before robust averaging.
			 */
			float sample_m = fabsf((tof / 2.0f) * SPEED_OF_LIGHT_NM_PER_S);

			if (isfinite(sample_m)) {
				rtt_distance_samples[candidate_count++] = sample_m;
			}
		}
	}
	*valid_samples = candidate_count;

	if (candidate_count < MIN_RTT_SAMPLES) {
		return false;
	}

	/* Median/MAD trimming is deliberately independent from the final mean:
	 * it rejects occasional valid-looking controller timings without imposing
	 * an absolute calibration model on RTT.
	 */
	float median_m = median_copy(rtt_distance_samples, candidate_count);
	*distance_m = median_m;
	for (uint16_t i = 0; i < candidate_count; i++) {
		rtt_sort_scratch[i] = fabsf(rtt_distance_samples[i] - median_m);
	}
	insertion_sort(rtt_sort_scratch, candidate_count);
	float mad_m = (candidate_count & 1u) != 0u
			      ? rtt_sort_scratch[candidate_count / 2]
			      : (rtt_sort_scratch[(candidate_count / 2) - 1] +
				 rtt_sort_scratch[candidate_count / 2]) /
					2.0f;
	float threshold_m = fmaxf(RTT_OUTLIER_FLOOR_M,
				  RTT_OUTLIER_SIGMA * RTT_MAD_SCALE * mad_m);
	float result_m = 0.0f;
	uint16_t inlier_count = 0;

	for (uint16_t i = 0; i < candidate_count; i++) {
		if (fabsf(rtt_distance_samples[i] - median_m) <= threshold_m) {
			inlier_count++;
			result_m += (rtt_distance_samples[i] - result_m) / inlier_count;
		}
	}

	if (inlier_count < MIN_RTT_SAMPLES) {
		return false;
	}

	float squared_error_sum = 0.0f;

	for (uint16_t i = 0; i < candidate_count; i++) {
		if (fabsf(rtt_distance_samples[i] - median_m) <= threshold_m) {
			float error_m = rtt_distance_samples[i] - result_m;
			squared_error_sum += error_m * error_m;
		}
	}

	if (!isfinite(result_m) || result_m < MIN_VALID_DISTANCE_M ||
	    result_m > MAX_VALID_RTT_DISTANCE_M) {
		return false;
	}

	*distance_m = result_m;
	*stddev_m = sqrtf(squared_error_sum / (inlier_count - 1));
	*valid_samples = inlier_count;
	return isfinite(*stddev_m);
}

static void process_tone_info_data(struct processing_context *context,
			      struct bt_hci_le_cs_step_data_tone_info local_tone_info[],
			      struct bt_hci_le_cs_step_data_tone_info peer_tone_info[],
			      uint8_t channel, uint8_t antenna_permutation_index)
{
	for (uint8_t i = 0; i < (context->n_ap + 1); i++) {
		if (local_tone_info[i].extension_indicator != BT_HCI_LE_CS_NOT_TONE_EXT_SLOT ||
		    peer_tone_info[i].extension_indicator != BT_HCI_LE_CS_NOT_TONE_EXT_SLOT) {
			continue;
		}

		if (context->iq_sample_channel_data_index >= MAX_NUM_IQ_SAMPLES) {
			LOG_WRN("More IQ samples than size of iq_sample_channel_data array");
			return;
		}

		iq_sample_channel_data[context->iq_sample_channel_data_index].failed = false;
		iq_sample_channel_data[context->iq_sample_channel_data_index].channel = channel;
		iq_sample_channel_data[context->iq_sample_channel_data_index].antenna_permutation =
			antenna_permutation_index;
		iq_sample_channel_data[context->iq_sample_channel_data_index].local_iq_sample =
			bt_le_cs_parse_pct(local_tone_info[i].phase_correction_term);
		iq_sample_channel_data[context->iq_sample_channel_data_index].peer_iq_sample =
			bt_le_cs_parse_pct(peer_tone_info[i].phase_correction_term);

		if (local_tone_info[i].quality_indicator == BT_HCI_LE_CS_TONE_QUALITY_LOW ||
		    local_tone_info[i].quality_indicator == BT_HCI_LE_CS_TONE_QUALITY_UNAVAILABLE ||
		    peer_tone_info[i].quality_indicator == BT_HCI_LE_CS_TONE_QUALITY_LOW ||
		    peer_tone_info[i].quality_indicator == BT_HCI_LE_CS_TONE_QUALITY_UNAVAILABLE) {
			iq_sample_channel_data[context->iq_sample_channel_data_index].failed = true;
		}

		context->iq_sample_channel_data_index++;
	}
}

static void process_rtt_timing_data(struct processing_context *context,
			       struct bt_hci_le_cs_step_data_mode_1 *local_rtt_data,
			       struct bt_hci_le_cs_step_data_mode_1 *peer_rtt_data)
{
	if (context->rtt_timing_data_index >= MAX_NUM_RTT_SAMPLES) {
		LOG_WRN("More RTT samples processed than size of rtt_timing_data array");
		return;
	}
	rtt_timing_data[context->rtt_timing_data_index].failed = false;
	bool aa_failed = local_rtt_data->packet_quality_aa_check !=
				 BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
			 peer_rtt_data->packet_quality_aa_check !=
				 BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL;
	bool rssi_missing =
		local_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
		peer_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE;
	bool timing_missing =
		local_rtt_data->toa_tod_initiator ==
			(int16_t)BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE ||
		peer_rtt_data->tod_toa_reflector ==
			(int16_t)BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE;

	context->rtt_aa_failures += aa_failed ? 1u : 0u;
	context->rtt_rssi_missing += rssi_missing ? 1u : 0u;
	context->rtt_timing_missing += timing_missing ? 1u : 0u;
	if (aa_failed || rssi_missing || timing_missing) {
		rtt_timing_data[context->rtt_timing_data_index].failed = true;
	}

	if (context->role == BT_CONN_LE_CS_ROLE_INITIATOR) {
		rtt_timing_data[context->rtt_timing_data_index].toa_tod_initiator =
			local_rtt_data->toa_tod_initiator;
		rtt_timing_data[context->rtt_timing_data_index].tod_toa_reflector =
			peer_rtt_data->tod_toa_reflector;
	} else if (context->role == BT_CONN_LE_CS_ROLE_REFLECTOR) {
		rtt_timing_data[context->rtt_timing_data_index].tod_toa_reflector =
			local_rtt_data->tod_toa_reflector;
		rtt_timing_data[context->rtt_timing_data_index].toa_tod_initiator =
			peer_rtt_data->toa_tod_initiator;
	}

	context->rtt_timing_data_index++;
}

static bool process_step_data(struct bt_le_cs_subevent_step *local_step,
			      struct bt_le_cs_subevent_step *peer_step, void *user_data)
{
	struct processing_context *context = (struct processing_context *)user_data;

	if (local_step->mode == BT_CONN_LE_CS_MAIN_MODE_2) {
		struct bt_hci_le_cs_step_data_mode_2 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_2 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)peer_step->data;

		process_tone_info_data(context, local_step_data->tone_info,
				       peer_step_data->tone_info, local_step->channel,
				       local_step_data->antenna_permutation_index);

	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_1) {
		struct bt_hci_le_cs_step_data_mode_1 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_1 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_1 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_1 *)peer_step->data;

		process_rtt_timing_data(context, local_step_data, peer_step_data);

	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3) {
		/* Sounding-sequence RTT adds packet_pct1/packet_pct2 before the Mode 3
		 * antenna permutation and tone array. RTT fields stay at the beginning
		 * of both layouts, but PBR must use the matching structure or its tone
		 * data would be parsed eight bytes too early.
		 */
		if (context->rtt_type == BT_CONN_LE_CS_RTT_TYPE_32_BIT_SOUNDING ||
		    context->rtt_type == BT_CONN_LE_CS_RTT_TYPE_96_BIT_SOUNDING) {
			struct bt_hci_le_cs_step_data_mode_3_ss_rtt *local_step_data =
				(struct bt_hci_le_cs_step_data_mode_3_ss_rtt *)local_step->data;
			struct bt_hci_le_cs_step_data_mode_3_ss_rtt *peer_step_data =
				(struct bt_hci_le_cs_step_data_mode_3_ss_rtt *)peer_step->data;

			process_rtt_timing_data(
				context, (struct bt_hci_le_cs_step_data_mode_1 *)local_step_data,
				(struct bt_hci_le_cs_step_data_mode_1 *)peer_step_data);
			process_tone_info_data(context, local_step_data->tone_info,
					       peer_step_data->tone_info, local_step->channel,
					       local_step_data->antenna_permutation_index);
		} else {
			struct bt_hci_le_cs_step_data_mode_3 *local_step_data =
				(struct bt_hci_le_cs_step_data_mode_3 *)local_step->data;
			struct bt_hci_le_cs_step_data_mode_3 *peer_step_data =
				(struct bt_hci_le_cs_step_data_mode_3 *)peer_step->data;

			process_rtt_timing_data(
				context, (struct bt_hci_le_cs_step_data_mode_1 *)local_step_data,
				(struct bt_hci_le_cs_step_data_mode_1 *)peer_step_data);
			process_tone_info_data(context, local_step_data->tone_info,
					       peer_step_data->tone_info, local_step->channel,
					       local_step_data->antenna_permutation_index);
		}
	}

	return true;
}

bool estimate_distance(struct net_buf_simple *local_steps, struct net_buf_simple *peer_steps,
		       uint8_t n_ap, enum bt_conn_le_cs_role role,
		       enum bt_conn_le_cs_rtt_type rtt_type,
		       struct distance_estimate *estimate)
{
	if (local_steps == NULL || peer_steps == NULL || estimate == NULL) {
		return false;
	}

	memset(estimate, 0, sizeof(*estimate));

	struct processing_context context = {
		.rtt_timing_data_index = 0,
		.iq_sample_channel_data_index = 0,
		.n_ap = n_ap,
		.role = role,
		.rtt_type = rtt_type,
	};

	memset(rtt_timing_data, 0, sizeof(rtt_timing_data));
	memset(iq_sample_channel_data, 0, sizeof(iq_sample_channel_data));

	bt_ras_rreq_rd_subevent_data_parse(peer_steps, local_steps, context.role, NULL,
					   process_step_data, &context);

	estimate->phase_valid = estimate_distance_using_phase_slope(
		iq_sample_channel_data, context.iq_sample_channel_data_index,
		&estimate->phase_m, &estimate->phase_rmse_rad, &estimate->phase_samples);
	estimate->rtt_valid = estimate_distance_using_time_of_flight(
		context.rtt_timing_data_index, &estimate->rtt_m, &estimate->rtt_stddev_m,
		&estimate->rtt_samples);
	estimate->rtt_diagnostic_m = estimate->rtt_m;
	estimate->rtt_records = context.rtt_timing_data_index;
	estimate->rtt_aa_failures = context.rtt_aa_failures;
	estimate->rtt_rssi_missing = context.rtt_rssi_missing;
	estimate->rtt_timing_missing = context.rtt_timing_missing;
	estimate->valid = estimate->phase_valid || estimate->rtt_valid;

	if (!estimate->valid) {
		LOG_INF("A reliable distance estimate could not be computed.");
	} else {
		LOG_INF("Estimated distance to reflector:");
	}

	if (estimate->rtt_valid) {
		LOG_INF("- Round-Trip Timing method: %f meters (derived from %d samples, "
			 "stddev %f m)",
			(double)estimate->rtt_m, estimate->rtt_samples,
			(double)estimate->rtt_stddev_m);
	} else {
		LOG_WRN("RTT rejected: records=%u candidates=%u aa_fail=%u rssi_miss=%u "
			"time_miss=%u diagnostic=%f m", estimate->rtt_records,
			estimate->rtt_samples, estimate->rtt_aa_failures,
			estimate->rtt_rssi_missing, estimate->rtt_timing_missing,
			(double)estimate->rtt_diagnostic_m);
	}
	if (estimate->phase_valid) {
		LOG_INF("- Phase-Based Ranging method: %f meters (derived from %d samples, "
			 "fit RMSE %f rad)",
			(double)estimate->phase_m, estimate->phase_samples,
			(double)estimate->phase_rmse_rad);
	} else if (estimate->phase_samples > 0) {
		LOG_WRN("Rejected phase estimate: %f m, fit RMSE %f rad, %u samples",
			(double)estimate->phase_m, (double)estimate->phase_rmse_rad,
			estimate->phase_samples);
	}

	return estimate->valid;
}
