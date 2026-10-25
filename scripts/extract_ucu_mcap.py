#!/usr/bin/env python3
"""
Extract a UCU SLAM Dataset sequence (ROS 2 .mcap) into TUM format for ORB-SLAM3.
No ROS installation needed.

Output folder:
  rgb/<timestamp>.png   color frames
  rgb.txt               "timestamp rgb/<timestamp>.png"   (for mono_tum)
  imu.txt               "timestamp gx gy gz ax ay az"      (for later, mono-inertial)
  groundtruth.txt       "timestamp x y z qx qy qz qw"      (GNSS position, ENU)

Usage:
  python extract_ucu_mcap.py SEQ.mcap OUT_DIR            # extract everything
  python extract_ucu_mcap.py SEQ.mcap --list             # only list topics
  python extract_ucu_mcap.py SEQ.mcap OUT_DIR --max-frames 300   # quick test
"""
import argparse
import os
import sys

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def stamp(msg, log_time_ns):
    """Message header time in seconds; falls back to bag log time."""
    h = getattr(msg, "header", None)
    if h is not None and (h.stamp.sec or h.stamp.nanosec):
        return h.stamp.sec + h.stamp.nanosec * 1e-9
    return log_time_ns * 1e-9


def image_to_bgr(schema_name, msg):
    """Decode sensor_msgs/Image or sensor_msgs/CompressedImage to a BGR array."""
    if schema_name.endswith("CompressedImage"):
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)

    enc = msg.encoding.lower()
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        img = data.reshape(msg.height, msg.step)[:, : msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img.copy()
    if enc in ("rgba8", "bgra8"):
        img = data.reshape(msg.height, msg.step)[:, : msg.width * 4]
        img = img.reshape(msg.height, msg.width, 4)
        code = cv2.COLOR_RGBA2BGR if enc == "rgba8" else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(img, code)
    if enc in ("mono8", "8uc1"):
        img = data.reshape(msg.height, msg.step)[:, : msg.width]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if enc in ("yuv422", "yuyv", "yuv422_yuy2"):
        img = data.reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def list_topics(path):
    with open(path, "rb") as f:
        summary = make_reader(f).get_summary()
    if summary is None:
        sys.exit("No summary in the mcap file (file may be incomplete).")
    counts = summary.statistics.channel_message_counts if summary.statistics else {}
    print(f"{'topic':40s} {'type':45s} messages")
    for cid, ch in summary.channels.items():
        schema = summary.schemas.get(ch.schema_id)
        print(f"{ch.topic:40s} {schema.name if schema else '?':45s} {counts.get(cid, '?')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap")
    ap.add_argument("out", nargs="?")
    ap.add_argument("--list", action="store_true", help="only list topics and exit")
    ap.add_argument("--image-topic", default="/cam_color/image_raw")
    ap.add_argument("--imu-topic", default="/imu")
    ap.add_argument("--gt-topic", default="/pose")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after N frames (0 = all)")
    args = ap.parse_args()

    if args.list:
        list_topics(args.mcap)
        return
    if not args.out:
        ap.error("OUT_DIR is required unless --list is given")

    os.makedirs(os.path.join(args.out, "rgb"), exist_ok=True)
    f_rgb = open(os.path.join(args.out, "rgb.txt"), "w")
    f_imu = open(os.path.join(args.out, "imu.txt"), "w")
    f_gt = open(os.path.join(args.out, "groundtruth.txt"), "w")
    # mono_tum skips exactly 3 header lines
    f_rgb.write("# color images\n# extracted from %s\n# timestamp filename\n" % os.path.basename(args.mcap))
    f_imu.write("# timestamp gx gy gz ax ay az\n")
    f_gt.write("# timestamp tx ty tz qx qy qz qw\n")

    n_img = n_imu = n_gt = 0
    topics = [args.image_topic, args.imu_topic, args.gt_topic]
    with open(args.mcap, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, msg in reader.iter_decoded_messages(topics=topics, log_time_order=True):
            t = stamp(msg, message.log_time)
            if channel.topic == args.image_topic:
                img = image_to_bgr(schema.name, msg)
                name = f"rgb/{t:.6f}.png"
                cv2.imwrite(os.path.join(args.out, name), img)
                f_rgb.write(f"{t:.6f} {name}\n")
                n_img += 1
                if n_img % 500 == 0:
                    print(f"  {n_img} frames...", flush=True)
                if args.max_frames and n_img >= args.max_frames:
                    break
            elif channel.topic == args.imu_topic:
                w, a = msg.angular_velocity, msg.linear_acceleration
                f_imu.write(f"{t:.9f} {w.x:.9f} {w.y:.9f} {w.z:.9f} {a.x:.9f} {a.y:.9f} {a.z:.9f}\n")
                n_imu += 1
            elif channel.topic == args.gt_topic:
                p = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
                q = p.orientation
                qw = q.w if (q.x or q.y or q.z or q.w) else 1.0
                f_gt.write(f"{t:.6f} {p.position.x:.6f} {p.position.y:.6f} {p.position.z:.6f} "
                           f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {qw:.6f}\n")
                n_gt += 1

    for fh in (f_rgb, f_imu, f_gt):
        fh.close()
    print(f"Done: {n_img} frames, {n_imu} IMU samples, {n_gt} ground-truth poses -> {args.out}")
    if n_img == 0:
        print("WARNING: no images. Run with --list and pass the right --image-topic.")
    if n_gt == 0:
        print("WARNING: no ground truth. Run with --list and pass the right --gt-topic.")


if __name__ == "__main__":
    main()
