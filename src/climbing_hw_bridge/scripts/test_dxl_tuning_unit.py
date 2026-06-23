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


if __name__ == "__main__":
    unittest.main()
