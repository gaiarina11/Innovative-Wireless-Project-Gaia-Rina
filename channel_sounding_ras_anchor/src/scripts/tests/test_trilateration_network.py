"""Test host-side per parsing, solver e calibrazione RTLS."""

import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS_DIR))

import trilateration_network as rtls  # noqa: E402


def make_sample(anchor_id, distance_m, received_monotonic=10.0, sequence=1):
    return rtls.DistanceSample(
        anchor_id=anchor_id,
        sequence=sequence,
        device_time_ms=1000 + sequence,
        method="PBR",
        distance_m=distance_m,
        sample_count=32,
        quality="OK",
        received_monotonic=received_monotonic,
    )


class DistanceParserTests(unittest.TestCase):
    def test_parses_mode3_pbr_and_rtt_metrics(self):
        line = (
            "DIST_DATA:ANCHOR:2|SEQ:7|T_MS:1234|METHOD:PHASE|"
            "RAW_VAL:1.275|SAMPLES:29|QUALITY:OK|"
            "PBR_VALID:1|PBR_VAL:1.275|PBR_SAMPLES:29|PBR_RMSE:0.1234|"
            "RTT_VALID:1|RTT_VAL:1.050|RTT_SAMPLES:24|RTT_STD:0.180"
        )

        sample = rtls.parse_distance_line(line, received_monotonic=7.5)

        self.assertIsNotNone(sample)
        self.assertTrue(sample.pbr_valid)
        self.assertAlmostEqual(sample.pbr_distance_m, 1.275)
        self.assertAlmostEqual(sample.pbr_rmse_rad, 0.1234)
        self.assertTrue(sample.rtt_valid)
        self.assertAlmostEqual(sample.rtt_distance_m, 1.050)
        self.assertEqual(sample.rtt_samples, 24)
        self.assertAlmostEqual(sample.rtt_stddev_m, 0.180)

    def test_keeps_pbr_guard_when_uncalibrated_mode3_estimates_disagree(self):
        line = (
            "DIST_DATA:ANCHOR:0|SEQ:1|T_MS:1|METHOD:PHASE|"
            "RAW_VAL:2.500|SAMPLES:29|QUALITY:OK|"
            "PBR_VALID:1|PBR_VAL:2.500|PBR_SAMPLES:29|PBR_RMSE:0.1000|"
            "RTT_VALID:1|RTT_VAL:0.800|RTT_SAMPLES:24|RTT_STD:0.200"
        )
        sample = rtls.parse_distance_line(line)

        selected = rtls.select_distance_source(sample, "fused", fusion_guard_m=0.75)

        self.assertEqual(selected.method, "FUSED_PBR_GUARD")
        self.assertAlmostEqual(selected.distance_m, 2.5)

    def test_keeps_phase_when_mode3_estimates_agree(self):
        line = (
            "DIST_DATA:ANCHOR:0|SEQ:1|T_MS:1|METHOD:PHASE|"
            "RAW_VAL:1.100|SAMPLES:29|QUALITY:OK|"
            "PBR_VALID:1|PBR_VAL:1.100|PBR_SAMPLES:29|PBR_RMSE:0.1000|"
            "RTT_VALID:1|RTT_VAL:1.300|RTT_SAMPLES:24|RTT_STD:0.200"
        )
        sample = rtls.parse_distance_line(line)

        selected = rtls.select_distance_source(sample, "fused", fusion_guard_m=0.75)

        self.assertEqual(selected.method, "FUSED_PBR")
        self.assertAlmostEqual(selected.distance_m, 1.1)

    def test_parses_firmware_record_with_log_prefix(self):
        line = (
            "[00:00:01.000,000] <inf> app: "
            "DIST_DATA:ANCHOR:2|SEQ:42|T_MS:1234|METHOD:PBR|"
            "RAW_VAL:1.275|SAMPLES:16|QUALITY:OK"
        )

        sample = rtls.parse_distance_line(line, received_monotonic=7.5)

        self.assertIsNotNone(sample)
        self.assertEqual(sample.anchor_id, 2)
        self.assertEqual(sample.sequence, 42)
        self.assertAlmostEqual(sample.distance_m, 1.275)
        self.assertEqual(sample.received_monotonic, 7.5)

    def test_rejects_bad_quality(self):
        line = (
            "DIST_DATA:ANCHOR:0|SEQ:1|T_MS:1|METHOD:PBR|"
            "RAW_VAL:1.0|SAMPLES:8|QUALITY:INVALID"
        )
        self.assertIsNone(rtls.parse_distance_line(line))

    def test_parses_commanded_cs_status(self):
        status = rtls.parse_cs_status_line(
            "CS_STATUS:ANCHOR:1|STATE:ERROR|CODE:-5", received_monotonic=8.0
        )

        self.assertIsNotNone(status)
        self.assertEqual(status.anchor_id, 1)
        self.assertEqual(status.state, "ERROR")
        self.assertEqual(status.code, -5)
        self.assertEqual(status.received_monotonic, 8.0)


class PbrBranchTrackerTests(unittest.TestCase):
    def test_initializes_from_known_position_without_changing_calibration(self):
        tracker = rtls.PbrBranchTracker(
            rtls.DEFAULT_ANCHORS, (0.75, 0.85), max_step_m=0.4
        )

        distance, realigned = tracker.add(2, 2.50)

        self.assertTrue(realigned)
        self.assertAlmostEqual(distance, 0.35)

    def test_realigns_large_branch_jump_and_tracks_following_variation(self):
        tracker = rtls.PbrBranchTracker(
            rtls.DEFAULT_ANCHORS, (0.75, 0.85), max_step_m=0.4
        )
        tracker.add(2, 0.35)

        distance, realigned = tracker.add(2, 2.50)
        following, following_realigned = tracker.add(2, 2.60)

        self.assertTrue(realigned)
        self.assertAlmostEqual(distance, 0.35)
        self.assertFalse(following_realigned)
        self.assertAlmostEqual(following, 0.45)


class FakeReader:
    def __init__(self, anchor_id):
        self.anchor_id = anchor_id
        self.commands = []

    def send_command(self, command):
        self.commands.append(command)
        return True


class TokenCoordinatorTests(unittest.TestCase):
    def test_repeats_ping_when_initial_ready_response_is_missing(self):
        readers = [FakeReader(anchor_id) for anchor_id in rtls.ANCHOR_IDS]
        coordinator = rtls.CsTokenCoordinator(
            readers, timeout_s=5.0, retry_delay_s=2.0
        )

        for anchor_id in rtls.ANCHOR_IDS:
            coordinator.on_link_status(
                rtls.SerialLinkStatus(anchor_id, True, received_monotonic=1.0)
            )
        for anchor_id in (0, 2):
            coordinator.on_anchor_status(
                rtls.AnchorStatus(anchor_id, "READY", 0, received_monotonic=1.1)
            )

        coordinator.tick(3.1)

        self.assertEqual(readers[1].commands, ["CS_PING", "CS_PING"])
        self.assertEqual(readers[0].commands, ["CS_PING"])
        self.assertEqual(readers[2].commands, ["CS_PING"])
        self.assertIsNone(coordinator.active_anchor)

        coordinator.on_anchor_status(
            rtls.AnchorStatus(1, "READY", 0, received_monotonic=3.2)
        )
        coordinator.tick(3.3)

        self.assertEqual(coordinator.active_anchor, 0)
        self.assertEqual(readers[0].commands[-1], "CS_MEASURE")

    def test_grants_one_token_and_waits_for_ready_release(self):
        readers = [FakeReader(anchor_id) for anchor_id in rtls.ANCHOR_IDS]
        coordinator = rtls.CsTokenCoordinator(readers, timeout_s=5.0, retry_delay_s=0.1)

        for anchor_id in rtls.ANCHOR_IDS:
            coordinator.on_link_status(
                rtls.SerialLinkStatus(anchor_id, True, received_monotonic=1.0)
            )
            coordinator.on_anchor_status(
                rtls.AnchorStatus(anchor_id, "READY", 0, received_monotonic=1.1)
            )

        coordinator.tick(2.0)
        self.assertEqual(coordinator.active_anchor, 0)
        self.assertEqual(readers[0].commands[-1], "CS_MEASURE")
        self.assertFalse(coordinator.accept_sample(make_sample(1, 1.0), 2.1))
        self.assertTrue(coordinator.accept_sample(make_sample(0, 1.0), 2.2))

        coordinator.tick(2.3)
        self.assertIsNone(coordinator.active_anchor)
        coordinator.on_anchor_status(
            rtls.AnchorStatus(0, "READY", 0, received_monotonic=2.4)
        )
        coordinator.tick(2.51)

        self.assertEqual(coordinator.active_anchor, 1)
        self.assertEqual(readers[1].commands[-1], "CS_MEASURE")

    def test_error_advances_token_instead_of_starving_other_anchors(self):
        readers = [FakeReader(anchor_id) for anchor_id in rtls.ANCHOR_IDS]
        coordinator = rtls.CsTokenCoordinator(readers, timeout_s=5.0, retry_delay_s=0.1)

        for anchor_id in rtls.ANCHOR_IDS:
            coordinator.on_link_status(
                rtls.SerialLinkStatus(anchor_id, True, received_monotonic=1.0)
            )
            coordinator.on_anchor_status(
                rtls.AnchorStatus(anchor_id, "READY", 0, received_monotonic=1.1)
            )

        coordinator.tick(2.0)
        coordinator.on_anchor_status(
            rtls.AnchorStatus(0, "ERROR", -5, received_monotonic=2.1)
        )
        coordinator.on_anchor_status(
            rtls.AnchorStatus(0, "READY", 0, received_monotonic=2.2)
        )
        coordinator.tick(2.31)

        self.assertEqual(coordinator.active_anchor, 1)
        self.assertEqual(readers[1].commands[-1], "CS_MEASURE")


class SynchronizerTests(unittest.TestCase):
    def test_calibration_accepts_complete_serialized_snapshot_with_large_skew(self):
        synchronizer = rtls.SampleSynchronizer(rtls.ANCHOR_IDS)
        synchronizer.add(make_sample(0, 1.0, received_monotonic=1.0))
        synchronizer.add(make_sample(1, 1.1, received_monotonic=7.0))
        synchronizer.add(make_sample(2, 1.2, received_monotonic=13.0))

        self.assertIsNone(
            synchronizer.coherent_snapshot(13.0, max_age_s=6.0, max_skew_s=4.0)
        )
        snapshot = synchronizer.complete_snapshot()

        self.assertIsNotNone(snapshot)
        self.assertEqual(set(snapshot), set(rtls.ANCHOR_IDS))
        self.assertIsNone(synchronizer.complete_snapshot())

    def test_requires_one_new_sample_from_every_anchor(self):
        synchronizer = rtls.SampleSynchronizer(rtls.ANCHOR_IDS)
        for anchor_id in rtls.ANCHOR_IDS:
            synchronizer.add(make_sample(anchor_id, 1.0, 10.0 + 0.01 * anchor_id))

        first = synchronizer.coherent_snapshot(10.1, max_age_s=1.0, max_skew_s=0.1)
        self.assertIsNotNone(first)
        self.assertIsNone(
            synchronizer.coherent_snapshot(10.2, max_age_s=1.0, max_skew_s=0.1)
        )

        synchronizer.add(make_sample(0, 1.1, 10.2, sequence=2))
        self.assertIsNone(
            synchronizer.coherent_snapshot(10.2, max_age_s=1.0, max_skew_s=0.3)
        )


class PositionSolverTests(unittest.TestCase):
    def test_recovers_exact_known_position(self):
        expected = (1.1, 0.9)
        distances = {
            anchor_id: math.dist(expected, anchor)
            for anchor_id, anchor in rtls.DEFAULT_ANCHORS.items()
        }

        solution = rtls.solve_position(distances, rtls.DEFAULT_ANCHORS)

        self.assertIsNotNone(solution)
        self.assertAlmostEqual(solution.position[0], expected[0], places=7)
        self.assertAlmostEqual(solution.position[1], expected[1], places=7)
        self.assertLess(solution.rmse_m, 1e-8)

    def test_reports_residual_for_inconsistent_circles(self):
        distances = {0: 1.05, 1: 1.30, 2: 1.13}

        solution = rtls.solve_position(distances, rtls.DEFAULT_ANCHORS)

        self.assertIsNotNone(solution)
        self.assertGreater(solution.rmse_m, 0.2)


class RuntimeFilterTests(unittest.TestCase):
    def test_hampel_filter_rejects_meter_scale_outlier(self):
        distance_filter = rtls.RobustDistanceFilter(
            window=5, sigma_threshold=4.0, floor_m=0.20
        )
        for value in (1.00, 1.04, 0.98, 1.02, 1.01):
            self.assertIsNotNone(distance_filter.add(value))

        self.assertIsNone(distance_filter.add(4.5))
        self.assertEqual(distance_filter.outlier_count, 1)
        self.assertAlmostEqual(distance_filter.add(1.03), 1.02, places=2)

    def test_position_filter_uses_configured_ema_weight(self):
        position_filter = rtls.PositionFilter(alpha=0.25)
        self.assertEqual(position_filter.add((0.0, 0.0)), (0.0, 0.0))
        filtered = position_filter.add((1.0, 2.0))
        self.assertAlmostEqual(filtered[0], 0.25)
        self.assertAlmostEqual(filtered[1], 0.50)

    def test_distance_filter_reacquires_consistent_new_distance(self):
        distance_filter = rtls.RobustDistanceFilter(
            window=5, sigma_threshold=4.0, floor_m=0.20
        )
        for value in (1.00, 1.02, 0.98, 1.01, 1.00):
            distance_filter.add(value)

        self.assertIsNone(distance_filter.add(2.00))
        self.assertIsNone(distance_filter.add(2.04))
        reacquired = distance_filter.add(1.98)

        self.assertIsNotNone(reacquired)
        self.assertAlmostEqual(reacquired, 2.00, places=2)


class ValidationTests(unittest.TestCase):
    def test_computes_accuracy_precision_and_availability(self):
        collector = rtls.ValidationCollector(
            target_position=(1.0, 1.0), required_samples=2, warmup_samples=1
        )
        collector.reject("distance_outlier")
        collector.add((1.5, 1.5), (1.4, 1.4))  # warm-up escluso dalle metriche
        collector.add((1.1, 1.0), (1.1, 1.0))
        collector.add((0.9, 1.0), (0.9, 1.0))

        summary = collector.summary()

        self.assertTrue(collector.complete)
        self.assertEqual(summary["total_snapshots"], 4)
        self.assertEqual(summary["accepted_snapshots"], 3)
        self.assertAlmostEqual(summary["availability"], 0.75)
        self.assertAlmostEqual(summary["filtered"]["mean_error_m"], 0.1)
        self.assertAlmostEqual(summary["filtered"]["bias_x_m"], 0.0)
        self.assertEqual(summary["rejections"], {"distance_outlier": 1})

    def test_csv_contains_raw_filtered_and_rejected_measurements(self):
        snapshot = {
            anchor_id: make_sample(anchor_id, 1.0 + anchor_id * 0.1)
            for anchor_id in rtls.ANCHOR_IDS
        }
        distances = {
            anchor_id: sample.distance_m for anchor_id, sample in snapshot.items()
        }

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "measurements.csv"
            logger = rtls.MeasurementLogger(path, ground_truth=(0.5, 0.5))
            logger.log(
                snapshot,
                "distance_outlier",
                20.0,
                distances,
                distances,
                {0: 1.0, 1: 1.1},
            )
            logger.close()
            with path.open(newline="", encoding="utf-8") as csv_file:
                rows = list(csv.DictReader(csv_file))

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["status"], "distance_outlier")
        self.assertEqual(rows[0]["raw_distance_a2"], "1.200000")
        self.assertEqual(rows[0]["pbr_valid_a2"], "0")
        self.assertEqual(rows[0]["rtt_valid_a2"], "0")
        self.assertEqual(rows[0]["filtered_distance_a2"], "")
        self.assertEqual(rows[0]["ground_truth_x_m"], "0.500000")


class CalibrationTests(unittest.TestCase):
    def _synthetic_dataset(self, scale, offset_m):
        sessions = []
        for target in ((0.8, 0.6), (1.5, 1.2), (2.2, 0.7)):
            raw_distances = {}
            for anchor_id, anchor in rtls.DEFAULT_ANCHORS.items():
                true_distance = math.dist(target, anchor)
                raw = (true_distance - offset_m) / scale
                raw_distances[str(anchor_id)] = [raw] * 5
            sessions.append(
                {
                    "position_m": list(target),
                    "raw_distances_m": raw_distances,
                }
            )
        return {"version": 1, "sessions": sessions}

    def test_fits_scale_and_offset_per_anchor(self):
        expected_scale = 1.12
        expected_offset = 0.18
        dataset = self._synthetic_dataset(expected_scale, expected_offset)

        calibrations, statistics = rtls.fit_distance_calibrations(
            dataset, rtls.DEFAULT_ANCHORS
        )

        for anchor_id in rtls.ANCHOR_IDS:
            self.assertAlmostEqual(calibrations[anchor_id].scale, expected_scale)
            self.assertAlmostEqual(calibrations[anchor_id].offset_m, expected_offset)
            self.assertEqual(statistics[anchor_id]["model"], "scale+offset")
            self.assertLess(statistics[anchor_id]["fit_rmse_m"], 1e-10)

    def test_accepts_observed_low_positive_scale(self):
        dataset = self._synthetic_dataset(scale=0.42, offset_m=0.10)

        calibrations, statistics = rtls.fit_distance_calibrations(
            dataset, rtls.DEFAULT_ANCHORS
        )
        rtls.validate_calibration_fit(calibrations, statistics, max_fit_rmse_m=0.25)

        for anchor_id in rtls.ANCHOR_IDS:
            self.assertAlmostEqual(calibrations[anchor_id].scale, 0.42)
            self.assertAlmostEqual(calibrations[anchor_id].offset_m, 0.10)

    def test_single_point_uses_offset_only(self):
        target = (1.0, 1.0)
        raw_distances = {
            str(anchor_id): [math.dist(target, anchor) - 0.4] * 4
            for anchor_id, anchor in rtls.DEFAULT_ANCHORS.items()
        }
        dataset = {
            "version": 1,
            "sessions": [
                {"position_m": list(target), "raw_distances_m": raw_distances}
            ],
        }

        calibrations, statistics = rtls.fit_distance_calibrations(
            dataset, rtls.DEFAULT_ANCHORS
        )

        for anchor_id in rtls.ANCHOR_IDS:
            self.assertEqual(calibrations[anchor_id].scale, 1.0)
            self.assertAlmostEqual(calibrations[anchor_id].offset_m, 0.4)
            self.assertEqual(statistics[anchor_id]["model"], "offset-only")

    def test_rejects_non_identifiable_multi_point_fit(self):
        sessions = []
        for target in ((0.4, 0.3), (1.1, 0.3), (0.75, 0.85)):
            sessions.append(
                {
                    "position_m": list(target),
                    "raw_distances_m": {
                        str(anchor_id): [1.0] * 5 for anchor_id in rtls.ANCHOR_IDS
                    },
                }
            )

        with self.assertRaisesRegex(ValueError, "fit non identificabile"):
            rtls.fit_distance_calibrations(
                {"version": 1, "sessions": sessions}, rtls.DEFAULT_ANCHORS
            )

    def test_rejects_implausible_offset_and_fit_rmse(self):
        calibrations = {
            anchor_id: rtls.DistanceCalibration(1.0, 0.0)
            for anchor_id in rtls.ANCHOR_IDS
        }
        statistics = {
            anchor_id: {"fit_rmse_m": 0.01} for anchor_id in rtls.ANCHOR_IDS
        }
        calibrations[1] = rtls.DistanceCalibration(
            1.0, rtls.MAX_ABS_CALIBRATION_OFFSET_M + 0.2
        )

        with self.assertRaisesRegex(ValueError, "offset non plausibile"):
            rtls.validate_calibration_fit(calibrations, statistics, 0.25)

        calibrations[1] = rtls.DistanceCalibration(1.0, 0.0)
        statistics[2]["fit_rmse_m"] = 0.30
        with self.assertRaisesRegex(ValueError, "fit troppo disperso"):
            rtls.validate_calibration_fit(calibrations, statistics, 0.25)

    def test_build_does_not_persist_dataset_before_validation(self):
        snapshots = [
            {
                anchor_id: make_sample(anchor_id, 1.0)
                for anchor_id in rtls.ANCHOR_IDS
            }
        ]

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "dataset.json"
            dataset = rtls.build_calibration_dataset(path, (0.4, 0.3), snapshots)
            self.assertFalse(path.exists())
            rtls.save_calibration_dataset(path, dataset)
            self.assertTrue(path.exists())

    def test_collect_only_persists_session_without_changing_config(self):
        snapshots = [
            {
                anchor_id: make_sample(anchor_id, 1.0 + anchor_id * 0.1)
                for anchor_id in rtls.ANCHOR_IDS
            }
        ]

        with tempfile.TemporaryDirectory() as temp_dir:
            dataset_path = Path(temp_dir) / "campaign.json"
            config_path = Path(temp_dir) / "rtls.json"
            config_path.write_text("unchanged\n", encoding="utf-8")
            collector = rtls.CalibrationCollector((0.4, 0.3), 1)
            collector.add(snapshots[0])
            args = type(
                "Args",
                (),
                {
                    "calibration_dataset": dataset_path,
                    "config": config_path,
                },
            )()

            rtls._store_calibration_session(args, collector)
            dataset = json.loads(dataset_path.read_text(encoding="utf-8"))

            self.assertEqual(len(dataset["sessions"]), 1)
            self.assertEqual(dataset["sessions"][0]["position_m"], [0.4, 0.3])
            self.assertEqual(config_path.read_text(encoding="utf-8"), "unchanged\n")

    def test_round_trips_configuration(self):
        config = rtls.RtlsConfig(
            anchors=rtls.DEFAULT_ANCHORS,
            calibrations={
                anchor_id: rtls.DistanceCalibration(1.01, 0.1 * anchor_id)
                for anchor_id in rtls.ANCHOR_IDS
            },
            max_rmse_m=0.25,
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "rtls.json"
            rtls.save_rtls_config(path, config)
            loaded = rtls.load_rtls_config(path)
            payload = json.loads(path.read_text(encoding="utf-8"))

        self.assertEqual(loaded.anchors, rtls.DEFAULT_ANCHORS)
        self.assertAlmostEqual(loaded.calibrations[2].offset_m, 0.2)
        self.assertAlmostEqual(loaded.max_rmse_m, 0.25)
        self.assertEqual(payload["version"], 1)


if __name__ == "__main__":
    unittest.main()
