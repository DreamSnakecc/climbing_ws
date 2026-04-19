#!/usr/bin/env python3

import argparse
import json
import os
import sys


def load_records(file_path):
    records = []
    with open(file_path, "r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise RuntimeError("Invalid JSON on line %d: %s" % (line_number, exc))
    if not records:
        raise RuntimeError("No records found in %s" % file_path)
    return records


def series_from_records(records, path, default=None):
    values = []
    for record in records:
        current = record
        missing = False
        for key in path:
            if not isinstance(current, dict) or key not in current:
                missing = True
                break
            current = current[key]
        values.append(default if missing else current)
    return values


def relative_times(records):
    start = float(records[0].get("stamp", 0.0))
    return [float(record.get("stamp", start)) - start for record in records]


def first_available_joint_feedback(records):
    for record in records:
        joint_feedback = record.get("joint_feedback")
        if isinstance(joint_feedback, dict):
            return joint_feedback
    return None


def build_joint_position_series(records, joint_count):
    joint_positions = [[] for _ in range(joint_count)]
    for record in records:
        joint_feedback = record.get("joint_feedback") or {}
        positions = joint_feedback.get("position_rad") or []
        for joint_index in range(joint_count):
            joint_positions[joint_index].append(float(positions[joint_index]) if joint_index < len(positions) else float("nan"))
    return joint_positions


def build_phase_spans(records, times):
    spans = []
    if not records:
        return spans
    current_phase = records[0].get("inferred_phase", "UNKNOWN")
    start_time = times[0]
    for index in range(1, len(records)):
        phase = records[index].get("inferred_phase", "UNKNOWN")
        if phase != current_phase:
            spans.append((start_time, times[index], current_phase))
            current_phase = phase
            start_time = times[index]
    spans.append((start_time, times[-1], current_phase))
    return spans


def plot_log(records, output_path, title):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required to plot logs: %s" % exc)

    times = relative_times(records)
    target_x = series_from_records(records, ["target_center", "x"], float("nan"))
    target_y = series_from_records(records, ["target_center", "y"], float("nan"))
    target_z = series_from_records(records, ["target_center", "z"], float("nan"))
    actual_x = series_from_records(records, ["actual_center_estimate", "x"], float("nan"))
    actual_y = series_from_records(records, ["actual_center_estimate", "y"], float("nan"))
    actual_z = series_from_records(records, ["actual_center_estimate", "z"], float("nan"))
    target_normal = series_from_records(records, ["target_normal_from_nominal_m"], float("nan"))
    actual_normal = series_from_records(records, ["actual_normal_from_nominal_m"], float("nan"))
    joint_feedback = first_available_joint_feedback(records)
    if joint_feedback is None:
        raise RuntimeError("This log does not contain joint_feedback. Re-run the test with the updated logger.")

    motor_ids = joint_feedback.get("motor_ids") or []
    joint_positions = build_joint_position_series(records, len(motor_ids))
    phase_spans = build_phase_spans(records, times)
    phase_colors = {
        "NO_TARGET": "#f3f4f6",
        "SWING_FREE": "#dbeafe",
        "DETACH_OR_TANGENTIAL": "#d1fae5",
        "PRELOAD_COMPRESS": "#fef3c7",
        "COMPLIANT_SETTLE": "#fde68a",
        "ATTACHED_HOLD": "#fecaca",
        "SUPPORT": "#e5e7eb",
    }

    figure, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
    figure.suptitle(title)

    axes[0].plot(times, target_x, label="target x", color="#2563eb", linewidth=1.8)
    axes[0].plot(times, target_y, label="target y", color="#16a34a", linewidth=1.8)
    axes[0].plot(times, target_z, label="target z", color="#dc2626", linewidth=1.8)
    if not all(value != value for value in actual_x):
        axes[0].plot(times, actual_x, label="actual x", color="#1d4ed8", linestyle="--", linewidth=1.3)
    if not all(value != value for value in actual_y):
        axes[0].plot(times, actual_y, label="actual y", color="#15803d", linestyle="--", linewidth=1.3)
    if not all(value != value for value in actual_z):
        axes[0].plot(times, actual_z, label="actual z", color="#b91c1c", linestyle="--", linewidth=1.3)
    axes[0].set_ylabel("Foot Target Center (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(times, target_normal, label="target normal", color="#7c3aed", linewidth=1.8)
    if not all(value != value for value in actual_normal):
        axes[1].plot(times, actual_normal, label="actual normal", color="#ea580c", linestyle="--", linewidth=1.5)
    axes[1].set_ylabel("Normal From Nominal (m)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    for joint_index, motor_id in enumerate(motor_ids):
        axes[2].plot(times, joint_positions[joint_index], label="motor %s" % motor_id, linewidth=1.8)
    axes[2].set_ylabel("Joint Position (rad)")
    axes[2].set_xlabel("Time From Log Start (s)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right")

    for axis in axes:
        y_min, y_max = axis.get_ylim()
        for start_time, end_time, phase in phase_spans:
            color = phase_colors.get(phase, "#f3f4f6")
            axis.axvspan(start_time, end_time, color=color, alpha=0.12)
        axis.set_ylim(y_min, y_max)

    phase_labels_axis = axes[0].secondary_xaxis("top")
    tick_positions = []
    tick_labels = []
    for start_time, end_time, phase in phase_spans:
        tick_positions.append(0.5 * (start_time + end_time))
        tick_labels.append(phase)
    phase_labels_axis.set_xticks(tick_positions)
    phase_labels_axis.set_xticklabels(tick_labels, rotation=0, fontsize=9)
    phase_labels_axis.set_xlabel("Inferred Phase")

    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Plot swing-leg state machine JSONL logs.")
    parser.add_argument("log_path", help="Path to the JSONL log file generated by test_swing_leg_state_machine.py")
    parser.add_argument("--output", help="Output image path. Defaults to <log_path>.png")
    parser.add_argument("--title", help="Custom plot title")
    return parser


def main():
    args = build_arg_parser().parse_args()
    log_path = os.path.abspath(os.path.expanduser(args.log_path))
    if not os.path.isfile(log_path):
        raise RuntimeError("Log file not found: %s" % log_path)

    output_path = args.output
    if output_path is None:
        output_path = log_path + ".png"
    output_path = os.path.abspath(os.path.expanduser(output_path))

    records = load_records(log_path)
    title = args.title or os.path.basename(log_path)
    plot_log(records, output_path, title)
    print("Saved plot to %s" % output_path)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        sys.stderr.write("plot_swing_leg_state_machine_log.py failed: %s\n" % exc)
        sys.exit(1)