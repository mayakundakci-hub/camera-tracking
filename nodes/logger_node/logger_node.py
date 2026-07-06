#!/usr/bin/env python3
# =============================================================
# logger_node  (Python)
# Subscribes to "hand/validation" and appends rows to a CSV
# =============================================================

import csv
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
from config import load_config

# generated from proto/camera_tracking.proto (protoc --python_out)
import camera_tracking_pb2 as pb


def main():
    cfg = load_config()
    topic = cfg["ecal"]["topic_validation"]
    out_dir = Path(cfg["logger"]["output_dir"])
    flush_every = cfg["logger"]["flush_every_n_rows"]

    ecal_core.initialize(sys.argv, "logger_node")

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"validation_{datetime.now():%Y%m%d_%H%M%S}.csv"
    f = open(out_path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow([
        "timestamp", "valid", "fanuc_is_stub", "error_mm",
        "error_x_mm", "error_y_mm", "error_z_mm",
        "fanuc_x", "fanuc_y", "fanuc_z",
        "camera_x", "camera_y", "camera_z",
    ])
    print(f"[logger_node] Writing {out_path}")

    row_count = 0

    def on_msg(topic_name, msg, send_time):
        nonlocal row_count
        writer.writerow([
            msg.timestamp, msg.valid, msg.fanuc_is_stub, round(msg.error_mm, 4),
            round(msg.error_x_mm, 4), round(msg.error_y_mm, 4), round(msg.error_z_mm, 4),
            msg.pose_fanuc.pos_x, msg.pose_fanuc.pos_y, msg.pose_fanuc.pos_z,
            msg.pose_camera.pos_x, msg.pose_camera.pos_y, msg.pose_camera.pos_z,
        ])
        row_count += 1
        if row_count % flush_every == 0:
            f.flush()

    sub = ProtoSubscriber(topic, pb.ValidationPacket)
    sub.set_callback(on_msg)

    # SIGINT (Ctrl+C) and SIGBREAK (Windows CTRL_BREAK_EVENT, used by the
    # cameratracking launcher to stop this process individually) both just
    # raise SystemExit here -- the actual cleanup lives in `finally` below
    # so it runs on every exit path, not only this one.
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, lambda *_: sys.exit(0))

    try:
        while ecal_core.ok():
            time.sleep(0.1)
    finally:
        f.close()
        ecal_core.finalize()


if __name__ == "__main__":
    main()
