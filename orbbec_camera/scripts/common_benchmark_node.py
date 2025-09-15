#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Support: ROS2
name: common_benchmark_node.py
function: A ROS2 node to monitor and log the performance of an Orbbec camera node:
          frame rates, delays, CPU and RAM usage, packet/frame loss statistics.
usage:
  ros2 run orbbec_camera common_benchmark_node.py --run_time 20 --csv_file /tmp/cam_log.csv
"""

import argparse
import rclpy
from rclpy.node import Node
import psutil
import time
import csv
import os
from collections import defaultdict
from orbbec_camera_msgs.msg import DeviceStatus
from sensor_msgs.msg import Image
import sys

from tabulate import tabulate

CAMERA_NODE_NAMES = ["component_container", "orbbec_camera_node", "nodelet"]

# ----------------tool functions----------------
def parse_duration(s):
    # Parse duration strings like "10s", "5m", "1h", "2d" into seconds.
    if isinstance(s, (int, float)):
        return float(s)

    s = str(s).strip().lower()
    if s.endswith("s"):
        return float(s[:-1])
    elif s.endswith("m"):
        return float(s[:-1]) * 60
    elif s.endswith("h"):
        return float(s[:-1]) * 3600
    elif s.endswith("d"):
        return float(s[:-1]) * 86400
    else:
        return float(s)

def format_duration(seconds):
    seconds = int(seconds)
    days, seconds = divmod(seconds, 86400)
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)

    parts = []
    if days > 0:
        parts.append(f"{days}d")
    if hours > 0:
        parts.append(f"{hours}h")
    if minutes > 0:
        parts.append(f"{minutes}m")
    if seconds > 0 or not parts:
        parts.append(f"{seconds}s")
    return " ".join(parts)
# ----------------------------------------------

class TopicTracker:
    def __init__(self, logger=None):
        self.received = 0

        self.last_time = None
        self.drop_frames = 0

        self.logger = logger

    def on_msg(self, header, avg_fps):
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        self.received += 1

        if self.last_time is not None and avg_fps > 0:
            dt = stamp - self.last_time
            expected_interval = 1.0 / avg_fps
            if expected_interval > 0 and dt > 1.5 * expected_interval:
                self.drop_frames += 1

        self.last_time = stamp

    def frames_loss_rate(self):
        total = self.received + self.drop_frames
        if total <= 0:
            return 0.0
        return float(self.drop_frames) / total

    def reset(self):
        self.__init__(logger=self.logger)


class CameraMonitorNode(Node):
    def __init__(self, run_time, csv_file="camera_monitor_log.csv"):
        super().__init__("camera_monitor_node")

        self.run_time = run_time
        self.start_time = time.time()
        self.process = psutil.Process(os.getpid())
        self.first_data_collected = False
        self.node_name = ""

        self.connection_type = None
        self.disconnect_count = 0
        self.prev_online = True
        self.finished = False

        self.stats = defaultdict(lambda:
                                 {"count": 0, "sum": 0.0, "cur": 0.0, "avg": 0.0, "min": float("inf"), "max": float("-inf")})

        self.cpu_stats = {"cur": 0.0, "avg": 0.0, "min": float("inf"), "max": float("-inf"), "count": 0, "sum": 0.0}
        self.ram_stats = {"cur": 0.0, "avg": 0.0, "min": float("inf"), "max": float("-inf"), "count": 0, "sum": 0.0}

        self.trackers = {
            "color": TopicTracker(logger=self.get_logger()),
            "depth": TopicTracker(logger=self.get_logger())
        }

        # CSV
        self.csv_file = csv_file
        self.csv_fh = open(self.csv_file, "w", newline="")
        self.csv_writer = csv.writer(self.csv_fh)
        self.csv_writer.writerow([
            "time(s)", "connection_type", "disconnects",
            "color_fps_cur", "color_fps_avg", "color_fps_min", "color_fps_max",
            "color_delay_cur", "color_delay_avg", "color_delay_min", "color_delay_max",
            "depth_fps_cur", "depth_fps_avg", "depth_fps_min", "depth_fps_max",
            "depth_delay_cur", "depth_delay_avg", "depth_delay_min", "depth_delay_max",
            "cpu_cur", "cpu_avg", "cpu_min", "cpu_max",
            "ram_cur", "ram_avg", "ram_min", "ram_max",
            "color_frames_loss", "color_frames_loss_rate(%)",
            "depth_frames_loss", "depth_frames_loss_rate(%)"
        ])

        # subscriptions
        self.create_subscription(DeviceStatus, "/camera/device_status", self.status_callback, 5)

        self.create_subscription(Image, "/camera/color/image_raw", lambda msg: self.image_callback(msg, "color"), 5)
        self.create_subscription(Image, "/camera/depth/image_raw", lambda msg: self.image_callback(msg, "depth"), 5)

        # timer runs every 1s to update system stats, log csv and print status
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        if elapsed > self.run_time:
            self.finish()
            rclpy.shutdown()
            return

        cpu, ram, self.node_name = self.get_camera_stats()
        self.update_sys_stat(self.cpu_stats, cpu)
        self.update_sys_stat(self.ram_stats, ram)

        if self.first_data_collected:
            self.log_to_csv(elapsed)
            self.print_status()

    def finish(self):
        if self.finished:
            return
        self.finished = True

        elapsed = time.time() - self.start_time
        try:
            self.csv_fh.close()
        except Exception:
            pass
        print(f"Monitoring finished, it takes time: {format_duration(elapsed)}")
        print(f"CSV data is saved to: {self.csv_file}")

    def find_camera_node(self):
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = " ".join(proc.info.get('cmdline') or [])
                if any(name.lower() in cmdline.lower() for name in CAMERA_NODE_NAMES):
                    return proc
            except Exception:
                continue
        return None

    def get_camera_stats(self):
        proc = self.find_camera_node()
        if not proc:
            return 0.0, 0.0, "Not Found"
        try:
            procs = [proc] + proc.children(recursive=True)
            cpu = sum((p.cpu_percent(interval=None) for p in procs)) / max(1, psutil.cpu_count())
            mem_bytes = sum((p.memory_info().rss for p in procs))
            mem_mb = mem_bytes / (1024 * 1024)
            name = f"{proc.name()} + {len(procs)-1} child" if len(procs) > 1 else proc.name()
            return cpu, mem_mb, name
        except Exception:
            return 0.0, 0.0, "Error"

    def status_callback(self, msg: DeviceStatus):
        if not self.first_data_collected:
            self.first_data_collected = True

        self.connection_type = msg.connection_type
        if self.prev_online and not msg.device_online:
            self.disconnect_count += 1
            self.prev_online = msg.device_online
            return

        self.prev_online = msg.device_online

        # update stats from DeviceStatus message fields
        self.update_stats("color_fps", msg.color_frame_rate_cur, msg.color_frame_rate_min, msg.color_frame_rate_max, msg.color_frame_rate_avg)
        self.update_stats("color_delay", msg.color_delay_ms_cur, msg.color_delay_ms_min, msg.color_delay_ms_max, msg.color_delay_ms_avg)
        self.update_stats("depth_fps", msg.depth_frame_rate_cur, msg.depth_frame_rate_min, msg.depth_frame_rate_max, msg.depth_frame_rate_avg)
        self.update_stats("depth_delay", msg.depth_delay_ms_cur, msg.depth_delay_ms_min, msg.depth_delay_ms_max, msg.depth_delay_ms_avg)

    def image_callback(self, msg: Image, stream: str):
        if stream not in ("color", "depth"):
            return
        header = msg.header
        tracker = self.trackers[stream]
        tracker.on_msg(header, self.stats[f"{stream}_fps"]["avg"])

    def update_stats(self, key, cur, min_val, max_val, avg_val):
        if min_val <= 1e-3 or avg_val < 0:  # ignore invalid data
            return
        s = self.stats[key]
        s["cur"] = (cur)
        s["count"] += 1
        s["sum"] += avg_val
        s["avg"] = s["sum"] / s["count"] if s["count"] > 0 else 0.0
        s["min"] = min(s["min"], min_val)
        s["max"] = max(s["max"], max_val)

    def update_sys_stat(self, stat_dict, value):
        stat_dict["cur"] = value
        if value is None or value <= 0.0 or not self.prev_online:
            return

        stat_dict["count"] += 1
        stat_dict["sum"] += value
        stat_dict["avg"] = stat_dict["sum"] / stat_dict["count"] if stat_dict["count"] > 0 else 0.0
        stat_dict["min"] = min(stat_dict["min"], value)
        stat_dict["max"] = max(stat_dict["max"], value)

    def log_to_csv(self, elapsed):
        if not self.prev_online:
            self.csv_writer.writerow([
                round(elapsed, 2), self.connection_type, self.disconnect_count, *["N/A"] * 28
            ])
            return

        color_tracker = self.trackers["color"]
        depth_tracker = self.trackers["depth"]
        color_frames_loss = color_tracker.drop_frames
        color_frames_loss_rate = round(color_tracker.frames_loss_rate() * 100.0, 2)
        depth_frames_loss = depth_tracker.drop_frames
        depth_frames_loss_rate = round(depth_tracker.frames_loss_rate() * 100.0, 2)

        # guard: if stats keys missing, use 0
        def safe(k):
            v = self.stats.get(k, {})
            return round(v.get("cur", 0.0), 2), round(v.get("avg", 0.0), 2), round(v.get("min", 0.0), 2), round(v.get("max", 0.0), 2)

        color_fps_cur, color_fps_avg, color_fps_min, color_fps_max = safe("color_fps")
        color_delay_cur, color_delay_avg, color_delay_min, color_delay_max = safe("color_delay")
        depth_fps_cur, depth_fps_avg, depth_fps_min, depth_fps_max = safe("depth_fps")
        depth_delay_cur, depth_delay_avg, depth_delay_min, depth_delay_max = safe("depth_delay")

        self.csv_writer.writerow([
            round(elapsed, 2), self.connection_type, self.disconnect_count,
            color_fps_cur, color_fps_avg, color_fps_min, color_fps_max,
            color_delay_cur, color_delay_avg, color_delay_min, color_delay_max,
            depth_fps_cur, depth_fps_avg, depth_fps_min, depth_fps_max,
            depth_delay_cur, depth_delay_avg, depth_delay_min, depth_delay_max,
            round(self.cpu_stats["cur"], 2), round(self.cpu_stats["avg"], 2), round(self.cpu_stats["min"], 2), round(self.cpu_stats["max"], 2),
            round(self.ram_stats["cur"], 2), round(self.ram_stats["avg"], 2), round(self.ram_stats["min"], 2), round(self.ram_stats["max"], 2),
            color_frames_loss, color_frames_loss_rate,
            depth_frames_loss, depth_frames_loss_rate
        ])

    def print_status(self):
        def format_stats(s):
            return f"{s['cur']:.2f}", f"{s['avg']:.2f}", f"{s['min']:.2f}", f"{s['max']:.2f}"

        rows = []
        for stream in ["color", "depth"]:
            fps_key = f"{stream}_fps"
            delay_key = f"{stream}_delay"
            topic_name = f"{stream}/image_raw"
            if not self.prev_online:
                rows.append([topic_name, *["N/A"] * 10])
            else:
                fps_vals = format_stats(self.stats[fps_key]) if self.stats[fps_key]["count"] > 0 else ("0.00","0.00","0.00","0.00")
                delay_vals = format_stats(self.stats[delay_key]) if self.stats[delay_key]["count"] > 0 else ("0.00","0.00","0.00","0.00")
                tracker = self.trackers[stream]

                frames_loss = tracker.drop_frames
                frames_loss_rate = round(tracker.frames_loss_rate() * 100.0, 2)
                rows.append([topic_name, *fps_vals, *delay_vals, frames_loss, frames_loss_rate])

        header_bottom = ["Option", "fps_cur", "fps_avg", "fps_min", "fps_max", "delay_cur(ms)", "delay_avg(ms)", "delay_min(ms)", "delay_max(ms)", "Pub_lost_count", "Pub_lost_rate(%)"]

        os.system("clear")
        print("Orbbec Camera Benchmark\n")
        print(tabulate([header_bottom] + rows, tablefmt="fancy_grid"))

        sys_rows = []
        if not self.prev_online:
            cpu_vals = (round(self.cpu_stats['cur'], 2), "N/A", "N/A", "N/A")
            ram_vals = (round(self.ram_stats['cur'], 2), "N/A", "N/A", "N/A")
        else:
            cpu_vals = format_stats(self.cpu_stats)
            ram_vals = format_stats(self.ram_stats)

        sys_rows.append(["CPU Usage (%)", *cpu_vals])
        sys_rows.append(["RAM Usage (MB)", *ram_vals])

        print(f"\n\n(CPU & RAM) Camera Node: {self.node_name}\n")
        print(tabulate(sys_rows, headers=["Option", "cur", "avg", "min", "max"], tablefmt="fancy_grid"))
        print("\nconnection_type: %s\nstatus_online: %s\ndisconnect_count: %d" % (self.connection_type, self.prev_online, self.disconnect_count))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run_time", type=str, default="10s", help="Total run time for monitoring, e.g., 10s, 5m, 1h.")
    parser.add_argument("--csv_file", type=str, default="camera_monitor_log.csv")
    cli_args, _ = parser.parse_known_args(argv)

    rclpy.init(args=argv)
    run_time = parse_duration(cli_args.run_time)
    node = CameraMonitorNode(run_time, cli_args.csv_file)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()
    finally:
        try:
            node.csv_fh.close()
        except Exception:
            pass
        node.destroy_node()

if __name__ == "__main__":
    main()
