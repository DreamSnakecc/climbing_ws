#!/usr/bin/env python3

import csv
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(__file__))

import tune_dxl_position as tuner


class DxlTuningUnitTest(unittest.TestCase):
    def test_step_metrics_and_score(self):
        directory = tempfile.mkdtemp(prefix="dxl_tuning_step_")
        self.addCleanup(shutil.rmtree, directory)
        path = os.path.join(directory, "step.csv")
        fields = [
            "elapsed_s", "phase", "target_tick", "actual_tick", "error_tick",
            "current_a", "feedback_age_s",
        ]
        with open(path, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows([
                {"elapsed_s": "0.00", "phase": "ramp_out", "target_tick": "100", "actual_tick": "0", "error_tick": "-100", "current_a": "0.2", "feedback_age_s": "0.01"},
                {"elapsed_s": "0.20", "phase": "ramp_out", "target_tick": "100", "actual_tick": "85", "error_tick": "-15", "current_a": "0.4", "feedback_age_s": "0.01"},
                {"elapsed_s": "0.40", "phase": "settle", "target_tick": "100", "actual_tick": "98", "error_tick": "-2", "current_a": "0.3", "feedback_age_s": "0.01"},
            ])
        metric = tuner.parse_step_csv(path, 10.0, 0.15)
        self.assertTrue(metric["safe"])
        self.assertEqual(metric["response_error_tick"], 15.0)
        combined = tuner.combine_step_metrics([metric, metric], 2.0, 100, 10.0)
        self.assertTrue(combined["safe"])
        self.assertIsNotNone(combined["score"])

    def test_step_overshoot_is_hard_failure(self):
        directory = tempfile.mkdtemp(prefix="dxl_tuning_overshoot_")
        self.addCleanup(shutil.rmtree, directory)
        path = os.path.join(directory, "step.csv")
        fields = [
            "elapsed_s", "phase", "target_tick", "actual_tick", "error_tick",
            "current_a", "feedback_age_s",
        ]
        with open(path, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows([
                {"elapsed_s": "0.00", "phase": "ramp_out", "target_tick": "100", "actual_tick": "0", "error_tick": "-100", "current_a": "0.2", "feedback_age_s": "0.01"},
                {"elapsed_s": "0.20", "phase": "ramp_out", "target_tick": "100", "actual_tick": "103", "error_tick": "3", "current_a": "0.3", "feedback_age_s": "0.01"},
                {"elapsed_s": "0.40", "phase": "settle", "target_tick": "100", "actual_tick": "100", "error_tick": "0", "current_a": "0.2", "feedback_age_s": "0.01"},
            ])
        metric = tuner.parse_step_csv(path, 3.0, 0.10, 2.0, 1)
        self.assertFalse(metric["safe"])
        self.assertFalse(metric["overshoot_safe"])

    def test_crawl_metric_comparison(self):
        baseline = {
            "fault_seen": False,
            "motors": {"8": {"p95_abs_error_tick": 100.0}},
            "legs": {"lr": {"p95_total_error_mm": 10.0}},
        }
        better = {
            "fault_seen": False,
            "motors": {"8": {"p95_abs_error_tick": 80.0}},
            "legs": {"lr": {"p95_total_error_mm": 8.0}},
        }
        worse = {
            "fault_seen": False,
            "motors": {"8": {"p95_abs_error_tick": 120.0}},
            "legs": {"lr": {"p95_total_error_mm": 8.0}},
        }
        self.assertTrue(tuner.compare_crawl_metrics(baseline, better)["passed"])
        self.assertFalse(tuner.compare_crawl_metrics(baseline, worse)["passed"])

    def test_crawl_csv_parser(self):
        directory = tempfile.mkdtemp(prefix="dxl_tuning_crawl_")
        self.addCleanup(shutil.rmtree, directory)
        path = os.path.join(directory, "crawl.csv")
        fields = [
            "mission_state", "lr_phase", "lr_cmd_x", "lr_cmd_y", "lr_cmd_z",
            "lr_ujc_x", "lr_ujc_y", "lr_ujc_z", "motor_8_error_tick",
            "motor_8_actual_current_a",
        ]
        with open(path, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows([
                {"mission_state": "CLIMB", "lr_phase": "TRANSFER", "lr_cmd_x": "0", "lr_cmd_y": "0", "lr_cmd_z": "-0.1", "lr_ujc_x": "-0.3", "lr_ujc_y": "0.3", "lr_ujc_z": "-0.1", "motor_8_error_tick": "20", "motor_8_actual_current_a": "0.8"},
                {"mission_state": "CLIMB", "lr_phase": "TRANSFER", "lr_cmd_x": "0", "lr_cmd_y": "0", "lr_cmd_z": "-0.1", "lr_ujc_x": "-0.3", "lr_ujc_y": "0.3", "lr_ujc_z": "-0.1", "motor_8_error_tick": "30", "motor_8_actual_current_a": "1.0"},
            ])
        config = {
            "gait_controller": {"base_radius": 203.06, "nominal_x": 127.75, "nominal_y": 0.0},
            "legs": {"lr": {"hip_yaw_deg": 135.0, "motor_ids": [14, 7, 8]}},
        }
        metrics = tuner.parse_crawl_metrics(path, config)
        self.assertFalse(metrics["fault_seen"])
        self.assertEqual(metrics["motors"]["8"]["p95_abs_error_tick"], 30.0)
        self.assertEqual(metrics["motors"]["8"]["p95_current_a"], 1.0)
        self.assertIsNotNone(metrics["legs"]["lr"]["p95_total_error_mm"])

    def test_commit_preserves_yaml_shape(self):
        directory = tempfile.mkdtemp(prefix="dxl_tuning_commit_")
        self.addCleanup(shutil.rmtree, directory)
        source = os.path.join(os.path.dirname(__file__), "..", "..", "climbing_description", "config", "robot.yaml")
        robot_config = os.path.join(directory, "robot.yaml")
        shutil.copy2(os.path.abspath(source), robot_config)
        candidates = {
            "left_board": {"2": {"p": 1300, "i": 0, "d": 20, "velocity": 250, "acceleration": 125}},
            "right_board": {"4": {"p": 1300, "i": 0, "d": 20, "velocity": 250, "acceleration": 125}},
        }
        backup = tuner.write_committed_tuning(robot_config, candidates, directory)
        self.assertTrue(os.path.isfile(backup))
        config = tuner.load_yaml(robot_config)
        self.assertEqual(config["left_board"]["position_gains"]["2"]["p"], 1300)
        self.assertEqual(config["left_board"]["position_profiles"]["2"]["velocity"], 250)
        self.assertEqual(config["right_board"]["position_gains"]["4"]["d"], 20)

    def test_single_leg_endpoint_parser(self):
        directory = tempfile.mkdtemp(prefix="dxl_tuning_leg_")
        self.addCleanup(shutil.rmtree, directory)
        path = os.path.join(directory, "single_leg.csv")
        fields = [
            "servo_gate_enabled", "servo_timed_out", "servo_last_pass_phase_id",
            "servo_last_pass_sequence", "servo_wait_s", "servo_phase_id",
            "servo_error_joint1_tick", "servo_error_joint2_tick", "servo_error_joint3_tick",
            "servo_error_joint4_tick",
            "servo_last_pass_error_joint1_tick", "servo_last_pass_error_joint2_tick", "servo_last_pass_error_joint3_tick",
            "servo_last_pass_error_joint4_tick",
        ]
        with open(path, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows([
                {"servo_gate_enabled": "1", "servo_timed_out": "0", "servo_last_pass_phase_id": "5", "servo_last_pass_sequence": "1", "servo_wait_s": "0", "servo_phase_id": "6", "servo_error_joint1_tick": "0", "servo_error_joint2_tick": "0", "servo_error_joint3_tick": "0", "servo_error_joint4_tick": "0", "servo_last_pass_error_joint1_tick": "1", "servo_last_pass_error_joint2_tick": "-2", "servo_last_pass_error_joint3_tick": "3", "servo_last_pass_error_joint4_tick": "-1"},
                {"servo_gate_enabled": "1", "servo_timed_out": "1", "servo_last_pass_phase_id": "5", "servo_last_pass_sequence": "1", "servo_wait_s": "3.1", "servo_phase_id": "6", "servo_error_joint1_tick": "2", "servo_error_joint2_tick": "-15", "servo_error_joint3_tick": "1", "servo_error_joint4_tick": "4", "servo_last_pass_error_joint1_tick": "1", "servo_last_pass_error_joint2_tick": "-2", "servo_last_pass_error_joint3_tick": "3", "servo_last_pass_error_joint4_tick": "-1"},
            ])
        metrics = tuner.parse_single_leg_csv(path, 10.0)
        self.assertFalse(metrics["passed"])
        self.assertEqual(metrics["endpoint_count"], 1)
        self.assertEqual(metrics["terminal_errors_tick"], [2.0, -15.0, 1.0, 4.0])
        self.assertEqual(tuner.failed_leg_motor_id(metrics, [11, 1, 2, 15], 10.0), 1)

    def test_single_leg_candidate_selection(self):
        baseline = {"passed": False, "timed_out": True, "endpoint_count": 0, "terminal_max_abs_error_tick": 15.0}
        improved = {"passed": False, "timed_out": True, "endpoint_count": 1, "terminal_max_abs_error_tick": 9.0}
        worse = {"passed": False, "timed_out": True, "endpoint_count": 0, "terminal_max_abs_error_tick": 16.0}
        self.assertTrue(tuner.leg_endpoint_improves(improved, baseline))
        self.assertFalse(tuner.leg_endpoint_improves(worse, baseline))
        original = {"p": 800, "i": 0, "d": 0, "velocity": 200, "acceleration": 100}
        candidates = tuner.leg_endpoint_candidates(original, original)
        labels = [item[0] for item in candidates]
        self.assertIn("p_600", labels)
        self.assertIn("i_200", labels)
        self.assertIn("d_128", labels)
        self.assertIn("velocity_300", labels)
        self.assertIn("acceleration_600", labels)

    def test_default_motor_list_is_4dof(self):
        self.assertEqual(len(tuner.DEFAULT_MOTOR_IDS), 16)
        self.assertEqual(set([15, 16, 17, 18]).issubset(set(tuner.DEFAULT_MOTOR_IDS)), True)

    def test_single_leg_tracking_requires_two_clean_cycles(self):
        directory = tempfile.mkdtemp(prefix="dxl_tuning_leg_tracking_")
        self.addCleanup(shutil.rmtree, directory)
        path = os.path.join(directory, "single_leg.csv")
        fields = ["servo_phase_id", "cmd_x", "cmd_y", "cmd_z", "ujc_x", "ujc_y", "ujc_z"]
        rows = []
        for _ in range(2):
            for phase_id, start, target in [
                    (5, [0.0, 0.0, -0.20], [0.0, 0.0, -0.19]),
                    (6, [0.0, 0.0, -0.19], [0.02, 0.0, -0.19]),
                    (2, [0.02, 0.0, -0.19], [0.02, 0.0, -0.20])]:
                for point in (start, target):
                    rows.append({
                        "servo_phase_id": phase_id,
                        "cmd_x": point[0], "cmd_y": point[1], "cmd_z": point[2],
                        "ujc_x": point[0], "ujc_y": point[1], "ujc_z": point[2],
                    })
        with open(path, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        config = {
            "gait_controller": {"base_radius": 0.0, "nominal_x": 0.0, "nominal_y": 0.0},
            "legs": {"lf": {"hip_yaw_deg": 0.0, "motor_ids": [11, 1, 2, 15]}},
            "position_autotune": {"minimum_cycles_per_leg": 2},
        }
        metrics = tuner.parse_single_leg_tracking(path, config, "lf")
        self.assertTrue(metrics["passed"])
        self.assertEqual(metrics["phases"]["TRANSFER"]["cycle_count"], 2)

    def test_motor_model_current_cap(self):
        config = {
            "dynamixel_telemetry": {
                "torque_model_by_motor": {"15": "xw430"},
                "torque_models": {"xw430": {"max_current_a": 1.22}},
            },
        }
        self.assertEqual(tuner.motor_current_limit_a(config, 15, 4.8), 1.22)

    def test_time_based_profile_is_rejected(self):
        tuning = {"operating_mode": 3, "drive_mode": 4}
        with self.assertRaises(RuntimeError):
            tuner.DxlAutoTuner._validate_tuning_mode(15, tuning)


if __name__ == "__main__":
    unittest.main()
