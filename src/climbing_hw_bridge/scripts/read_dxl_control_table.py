#!/usr/bin/env python3

import argparse
import csv
import os
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


PROTOCOL_VERSION = 2.0

REGISTERS = [
    ("operating_mode", 11, 1),
    ("pwm_limit", 36, 2),
    ("current_limit", 38, 2),
    ("position_d_gain", 80, 2),
    ("position_i_gain", 82, 2),
    ("position_p_gain", 84, 2),
    ("profile_acceleration", 108, 4),
    ("profile_velocity", 112, 4),
]


def parse_ids(value):
    if value is None or str(value).strip() == "":
        return []
    return [int(part.strip()) for part in str(value).split(",") if part.strip()]


def read_register(packet_handler, port_handler, motor_id, address, size):
    if size == 1:
        value, comm_result, dxl_error = packet_handler.read1ByteTxRx(
            port_handler, motor_id, address
        )
    elif size == 2:
        value, comm_result, dxl_error = packet_handler.read2ByteTxRx(
            port_handler, motor_id, address
        )
    elif size == 4:
        value, comm_result, dxl_error = packet_handler.read4ByteTxRx(
            port_handler, motor_id, address
        )
    else:
        raise ValueError("unsupported register size: %s" % size)

    if comm_result != COMM_SUCCESS:
        return None, packet_handler.getTxRxResult(comm_result)
    if dxl_error != 0:
        return None, packet_handler.getRxPacketError(dxl_error)
    return int(value), ""


def read_board(board_name, device_name, baudrate, motor_ids):
    rows = []
    if not motor_ids:
        return rows

    port_handler = PortHandler(device_name)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        raise RuntimeError(
            "%s: failed to open %s. Is multi_dxl_node still using the port?"
            % (board_name, device_name)
        )
    try:
        if not port_handler.setBaudRate(int(baudrate)):
            raise RuntimeError("%s: failed to set baudrate %s" % (board_name, baudrate))

        for motor_id in motor_ids:
            row = {
                "board": board_name,
                "device": device_name,
                "baudrate": int(baudrate),
                "motor_id": int(motor_id),
            }
            errors = []
            for register_name, address, size in REGISTERS:
                value, error = read_register(
                    packet_handler,
                    port_handler,
                    int(motor_id),
                    int(address),
                    int(size),
                )
                row[register_name] = "" if value is None else value
                if error:
                    errors.append("%s: %s" % (register_name, error))
            row["errors"] = "; ".join(errors)
            rows.append(row)
    finally:
        port_handler.closePort()

    return rows


def print_table(rows):
    columns = [
        "board",
        "motor_id",
        "operating_mode",
        "pwm_limit",
        "current_limit",
        "position_p_gain",
        "position_i_gain",
        "position_d_gain",
        "profile_velocity",
        "profile_acceleration",
        "errors",
    ]
    widths = {}
    for column in columns:
        widths[column] = max(
            len(column),
            max([len(str(row.get(column, ""))) for row in rows] or [0]),
        )

    header = "  ".join(column.ljust(widths[column]) for column in columns)
    print(header)
    print("  ".join("-" * widths[column] for column in columns))
    for row in rows:
        print("  ".join(str(row.get(column, "")).ljust(widths[column]) for column in columns))


def write_csv(rows, output_dir):
    output_dir = os.path.expanduser(output_dir)
    if not os.path.isabs(output_dir):
        workspace = os.environ.get(
            "CLIMBING_WS",
            os.environ.get("WORKSPACE_DIR", os.path.join(os.path.expanduser("~"), "climbing_ws")),
        )
        output_dir = os.path.join(workspace, output_dir)
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(
        output_dir,
        "dxl_control_table_%s.csv" % time.strftime("%Y%m%d_%H%M%S"),
    )
    columns = ["board", "device", "baudrate", "motor_id"] + [name for name, _, _ in REGISTERS] + ["errors"]
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read selected Dynamixel X-series control table parameters."
    )
    parser.add_argument("--left-port", default="/dev/ttyUSB_l")
    parser.add_argument("--right-port", default="/dev/ttyUSB_r")
    parser.add_argument("--baudrate", type=int, default=1000000)
    parser.add_argument("--left-ids", default="2,8")
    parser.add_argument("--right-ids", default="4,6")
    parser.add_argument("--output-dir", default="")
    return parser.parse_args()


def main():
    args = parse_args()
    rows = []
    rows.extend(read_board("left_board", args.left_port, args.baudrate, parse_ids(args.left_ids)))
    rows.extend(read_board("right_board", args.right_port, args.baudrate, parse_ids(args.right_ids)))
    if not rows:
        print("No motors selected.", file=sys.stderr)
        return 2
    print_table(rows)
    if args.output_dir:
        path = write_csv(rows, args.output_dir)
        print("\nCSV: %s" % path)
    return 1 if any(row.get("errors") for row in rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
