#!/usr/bin/env python3

import argparse
import csv
import os
import threading
import time

import rospy
from sensor_msgs.msg import JointState

from dynamixel_control.msg import SetPosition


DEFAULT_MOTOR_IDS = [2, 4, 6, 8]


class DxlPositionStepTester(object):
    def __init__(self, args):
        self.args = args
        self.motor_ids = parse_motor_ids(args.motor_ids)
        self.samples = {}
        self.lock = threading.Lock()
        self.current_lsb_ma = float(
            rospy.get_param("/dynamixel_telemetry/current_lsb_ma", 2.69)
        )
        self.t_zero = time.time()
        self.csv_file = None
        self.csv_writer = None
        self.results = []

        self.command_pub = rospy.Publisher("/set_position", SetPosition, queue_size=20)
        rospy.Subscriber(
            "/jetson/left_board/joint_state",
            JointState,
            self._joint_state_callback,
            callback_args="left_board",
            queue_size=20,
        )
        rospy.Subscriber(
            "/jetson/right_board/joint_state",
            JointState,
            self._joint_state_callback,
            callback_args="right_board",
            queue_size=20,
        )

    def _joint_state_callback(self, msg, board_name):
        now = time.time()
        with self.lock:
            for index, name in enumerate(msg.name):
                try:
                    motor_id = int(name)
                except ValueError:
                    continue
                position = float(msg.position[index]) if index < len(msg.position) else 0.0
                velocity = float(msg.velocity[index]) if index < len(msg.velocity) else 0.0
                effort = float(msg.effort[index]) if index < len(msg.effort) else 0.0
                self.samples[motor_id] = {
                    "board": board_name,
                    "position": position,
                    "velocity": velocity,
                    "effort_raw": effort,
                    "stamp": msg.header.stamp.to_sec() if msg.header.stamp else 0.0,
                    "wall_time": now,
                }

    def _latest_sample(self, motor_id):
        with self.lock:
            sample = self.samples.get(int(motor_id))
            return dict(sample) if sample is not None else None

    def _wait_for_feedback(self):
        rospy.loginfo(
            "test_dxl_position_step: waiting for board feedback for motors %s...",
            ",".join(str(mid) for mid in self.motor_ids),
        )
        deadline = time.time() + max(0.1, float(self.args.stream_timeout_s))
        while not rospy.is_shutdown() and time.time() < deadline:
            missing = [mid for mid in self.motor_ids if self._latest_sample(mid) is None]
            if not missing:
                return True
            rospy.sleep(0.05)
        rospy.logerr(
            "test_dxl_position_step: missing feedback for motors: %s",
            ",".join(str(mid) for mid in missing),
        )
        return False

    def _wait_for_command_subscribers(self):
        deadline = time.time() + max(0.1, float(self.args.stream_timeout_s))
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.command_pub.get_num_connections() > 0:
                return True
            rospy.sleep(0.05)
        rospy.logerr("test_dxl_position_step: /set_position has no subscribers")
        return False

    def _publish_target(self, motor_id, target_tick):
        msg = SetPosition()
        msg.id = int(motor_id)
        msg.position = int(round(target_tick))
        self.command_pub.publish(msg)

    def _open_csv(self):
        output_dir = os.path.expanduser(self.args.output_dir)
        if not os.path.isabs(output_dir):
            workspace = os.environ.get(
                "CLIMBING_WS",
                os.environ.get("WORKSPACE_DIR", os.path.join(os.path.expanduser("~"), "climbing_ws")),
            )
            output_dir = os.path.join(workspace, output_dir)
        os.makedirs(output_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(output_dir, "dxl_position_step_%s.csv" % stamp)
        self.csv_file = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "wall_time",
            "elapsed_s",
            "motor_id",
            "board",
            "phase",
            "step_index",
            "target_tick",
            "actual_tick",
            "error_tick",
            "abs_error_tick",
            "velocity_tick_s",
            "effort_raw",
            "current_a",
            "feedback_age_s",
        ])
        rospy.loginfo("test_dxl_position_step: writing CSV -> %s", path)
        return path

    def _write_sample(self, motor_id, phase, step_index, target_tick):
        sample = self._latest_sample(motor_id)
        if sample is None:
            return None
        actual = float(sample["position"])
        error = actual - float(target_tick)
        effort_raw = float(sample["effort_raw"])
        feedback_age_s = max(0.0, time.time() - float(sample["wall_time"]))
        row = [
            "%.6f" % time.time(),
            "%.6f" % (time.time() - self.t_zero),
            int(motor_id),
            sample["board"],
            phase,
            int(step_index),
            int(round(target_tick)),
            "%.3f" % actual,
            "%.3f" % error,
            "%.3f" % abs(error),
            "%.3f" % float(sample["velocity"]),
            "%.3f" % effort_raw,
            "%.6f" % (effort_raw * self.current_lsb_ma / 1000.0),
            "%.6f" % feedback_age_s,
        ]
        if self.csv_writer is not None:
            self.csv_writer.writerow(row)
        return {
            "actual": actual,
            "error": error,
            "abs_error": abs(error),
            "effort_raw": effort_raw,
            "current_a": effort_raw * self.current_lsb_ma / 1000.0,
            "feedback_age_s": feedback_age_s,
        }

    def _hold_target(self, motor_id, phase, step_index, target_tick, hold_s):
        self._publish_target(motor_id, target_tick)
        rate = rospy.Rate(max(1.0, float(self.args.sample_rate_hz)))
        deadline = time.time() + max(0.0, float(hold_s))
        last = None
        while not rospy.is_shutdown() and time.time() < deadline:
            last = self._write_sample(motor_id, phase, step_index, target_tick) or last
            rate.sleep()
        last = self._write_sample(motor_id, phase, step_index, target_tick) or last
        if self.csv_file is not None:
            self.csv_file.flush()
        return last

    def _target_for_step(self, home_tick, step_index, step_ticks):
        fraction = float(step_index) / max(1.0, float(self.args.ramp_steps))
        return int(round(float(home_tick) + float(step_ticks) * fraction))

    def _test_direction(self, motor_id, home_tick, step_ticks, direction_name):
        final_target = self._target_for_step(home_tick, int(self.args.ramp_steps), step_ticks)
        max_abs_error = 0.0
        max_abs_current = 0.0
        max_feedback_age_s = 0.0
        final_sample = None

        for step_index in range(1, int(self.args.ramp_steps) + 1):
            target = self._target_for_step(home_tick, step_index, step_ticks)
            rospy.loginfo(
                "test_dxl_position_step: ID %d %s ramp out step %d/%d target=%d",
                motor_id, direction_name, step_index, int(self.args.ramp_steps), target,
            )
            sample = self._hold_target(motor_id, "ramp_out", step_index, target, self.args.hold_s)
            if sample is not None:
                max_abs_error = max(max_abs_error, sample["abs_error"])
                max_abs_current = max(max_abs_current, abs(sample["current_a"]))
                max_feedback_age_s = max(max_feedback_age_s, sample["feedback_age_s"])
                final_sample = sample

        final_sample = self._hold_target(
            motor_id, "settle", int(self.args.ramp_steps), final_target, self.args.settle_s,
        ) or final_sample
        if final_sample is not None:
            max_abs_error = max(max_abs_error, final_sample["abs_error"])
            max_abs_current = max(max_abs_current, abs(final_sample["current_a"]))
            max_feedback_age_s = max(max_feedback_age_s, final_sample["feedback_age_s"])

        final_error = final_sample["error"] if final_sample is not None else float("inf")
        final_actual = final_sample["actual"] if final_sample is not None else float("nan")
        passed = abs(final_error) <= float(self.args.pass_error_ticks)

        for step_index in range(int(self.args.ramp_steps) - 1, -1, -1):
            target = self._target_for_step(home_tick, step_index, step_ticks)
            sample = self._hold_target(motor_id, "ramp_back", step_index, target, self.args.hold_s)
            if sample is not None:
                max_abs_error = max(max_abs_error, sample["abs_error"])
                max_abs_current = max(max_abs_current, abs(sample["current_a"]))
                max_feedback_age_s = max(max_feedback_age_s, sample["feedback_age_s"])

        return {
            "direction": direction_name,
            "final_target": final_target,
            "final_actual": final_actual,
            "final_error": final_error,
            "max_abs_error": max_abs_error,
            "max_abs_current": max_abs_current,
            "max_feedback_age_s": max_feedback_age_s,
            "passed": passed,
        }

    def _test_motor(self, motor_id):
        home_sample = self._latest_sample(motor_id)
        if home_sample is None:
            rospy.logerr("test_dxl_position_step: no sample for motor %d", motor_id)
            return {
                "motor_id": motor_id,
                "passed": False,
                "reason": "missing_feedback",
            }

        home_tick = int(round(home_sample["position"]))
        direction_results = []
        for direction in parse_directions(self.args.directions):
            signed_step_ticks = int(self.args.step_ticks) * direction
            direction_name = "positive" if signed_step_ticks >= 0 else "negative"
            rospy.loginfo(
                "test_dxl_position_step: ID %d home=%d %s final_target=%d",
                motor_id,
                home_tick,
                direction_name,
                self._target_for_step(home_tick, int(self.args.ramp_steps), signed_step_ticks),
            )
            direction_results.append(self._test_direction(
                motor_id, home_tick, signed_step_ticks, direction_name,
            ))
        last_result = direction_results[-1]
        return {
            "motor_id": motor_id,
            "home_tick": home_tick,
            "final_target": last_result["final_target"],
            "final_actual": last_result["final_actual"],
            "final_error": last_result["final_error"],
            "max_abs_error": max(result["max_abs_error"] for result in direction_results),
            "max_abs_current": max(result["max_abs_current"] for result in direction_results),
            "max_feedback_age_s": max(result["max_feedback_age_s"] for result in direction_results),
            "directions": direction_results,
            "passed": all(result["passed"] for result in direction_results),
            "reason": "ok" if all(result["passed"] for result in direction_results) else "final_error",
        }

    def _print_plan(self):
        print("Dynamixel position step test plan:")
        print("  motors: %s" % ",".join(str(mid) for mid in self.motor_ids))
        print("  command topic: /set_position")
        print("  step_ticks: %s (directions=%s)" % (self.args.step_ticks, self.args.directions))
        print("  ramp_steps: %s" % self.args.ramp_steps)
        print("  hold_s: %.3f" % float(self.args.hold_s))
        print("  settle_s: %.3f" % float(self.args.settle_s))
        print("  pass_error_ticks: %.3f" % float(self.args.pass_error_ticks))
        for motor_id in self.motor_ids:
            sample = self._latest_sample(motor_id)
            if sample is None:
                print("  ID %d: feedback missing" % motor_id)
                continue
            home = int(round(sample["position"]))
            print(
                "  ID %d: board=%s home=%d targets=%s"
                % (motor_id, sample["board"], home, ",".join(
                    str(self._target_for_step(home, int(self.args.ramp_steps), int(self.args.step_ticks) * direction))
                    for direction in parse_directions(self.args.directions)
                ))
            )

    def _confirm(self):
        if self.args.no_confirm:
            return True
        try:
            input(
                "\n>>> Confirm robot is supported, auto position/mode/current control is disabled, "
                "and motors %s can safely move +/- %d ticks. Press Enter to start. (Ctrl+C to cancel)\n>>> "
                % (",".join(str(mid) for mid in self.motor_ids), abs(int(self.args.step_ticks)))
            )
            return True
        except (EOFError, KeyboardInterrupt):
            rospy.loginfo("test_dxl_position_step: cancelled before start")
            return False

    def run(self):
        if int(self.args.ramp_steps) <= 0:
            rospy.logerr("test_dxl_position_step: --ramp-steps must be > 0")
            return 2
        if self.args.dry_run:
            self._print_plan()
            rospy.loginfo("test_dxl_position_step: dry-run complete; no commands sent")
            return 0
        if not self._wait_for_feedback():
            return 2
        self._print_plan()
        if not self._wait_for_command_subscribers():
            return 2
        if not self._confirm():
            return 0

        csv_path = self._open_csv()
        overall_pass = True
        try:
            for motor_id in self.motor_ids:
                result = self._test_motor(motor_id)
                self.results.append(result)
                overall_pass = overall_pass and bool(result["passed"])
        finally:
            if self.csv_file is not None:
                self.csv_file.close()

        rospy.loginfo("test_dxl_position_step: summary")
        for result in self.results:
            rospy.loginfo(
                "  ID %d: %s home=%s target=%s actual=%.1f final_error=%.1f "
                "max_error=%.1f max_current=%.3fA",
                int(result["motor_id"]),
                "PASS" if result["passed"] else "FAIL",
                result.get("home_tick", "?"),
                result.get("final_target", "?"),
                float(result.get("final_actual", float("nan"))),
                float(result.get("final_error", float("inf"))),
                float(result.get("max_abs_error", 0.0)),
                float(result.get("max_abs_current", 0.0)),
            )
        rospy.loginfo("test_dxl_position_step: CSV saved to %s", csv_path)
        return 0 if overall_pass else 1


def parse_motor_ids(value):
    if value is None or str(value).strip() == "":
        return list(DEFAULT_MOTOR_IDS)
    result = []
    for part in str(value).split(","):
        part = part.strip()
        if not part:
            continue
        result.append(int(part))
    if not result:
        raise argparse.ArgumentTypeError("at least one motor id is required")
    return result


def parse_directions(value):
    result = []
    for part in str(value).split(","):
        token = part.strip().lower()
        if token in ("+", "positive", "plus", "1"):
            result.append(1)
        elif token in ("-", "negative", "minus", "-1"):
            result.append(-1)
        else:
            raise argparse.ArgumentTypeError("directions must contain '+' and/or '-'")
    if not result:
        raise argparse.ArgumentTypeError("at least one direction is required")
    return result


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run direct /set_position ramp-step tracking tests for selected Dynamixel motors."
    )
    parser.add_argument("--motor-ids", default="2,4,6,8", help="Comma-separated motor IDs")
    parser.add_argument("--step-ticks", type=int, default=100, help="Final tick offset magnitude")
    parser.add_argument("--directions", default="+,-", help="Comma-separated test directions, default '+,-'")
    parser.add_argument("--ramp-steps", type=int, default=1, help="Number of ramp-out steps")
    parser.add_argument("--hold-s", type=float, default=0.6, help="Hold time for each ramp step")
    parser.add_argument("--settle-s", type=float, default=0.2, help="Extra hold at final target")
    parser.add_argument("--pass-error-ticks", type=float, default=10.0, help="Final absolute error threshold")
    parser.add_argument("--output-dir", default="test_logs", help="CSV output directory")
    parser.add_argument("--sample-rate-hz", type=float, default=50.0, help="CSV sample rate while holding targets")
    parser.add_argument("--stream-timeout-s", type=float, default=10.0, help="Feedback/topic wait timeout")
    parser.add_argument("--no-confirm", action="store_true", help="Skip interactive safety confirmation")
    parser.add_argument("--dry-run", action="store_true", help="Print plan and exit without sending commands")
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("test_dxl_position_step", anonymous=False)
    tester = DxlPositionStepTester(args)
    return tester.run()


if __name__ == "__main__":
    raise SystemExit(main())
