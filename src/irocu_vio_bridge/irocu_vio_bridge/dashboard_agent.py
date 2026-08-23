import csv
import json
import math
import os
import re
import socket
import time
import urllib.request
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import State, SysStatus
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Imu
from std_msgs.msg import String


class RateTracker:
    def __init__(self, size=80):
        self.times = deque(maxlen=size)

    def tick(self):
        self.times.append(time.time())

    def age(self):
        if not self.times:
            return None
        return time.time() - self.times[-1]

    def hz(self):
        if len(self.times) < 2:
            return 0.0
        dt = self.times[-1] - self.times[0]
        if dt <= 0.0:
            return 0.0
        return (len(self.times) - 1) / dt

    def alive(self, stale_seconds):
        age = self.age()
        if age is None:
            return False
        return age <= stale_seconds


class DashboardAgent(Node):
    def __init__(self):
        super().__init__("dashboard_agent")

        self.declare_parameter("dashboard_url", "http://10.57.124.90:9000/api/pi_status")
        self.declare_parameter("mac_receiver_health_url", "http://10.57.124.90:8000/health")
        self.declare_parameter("post_enabled", True)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("post_timeout", 0.8)
        self.declare_parameter("stale_seconds", 2.0)
        self.declare_parameter("mac_check_period_seconds", 5.0)

        self.declare_parameter("vio_topic", "/vio/odometry")
        self.declare_parameter("local_pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("state_topic", "/mavros/state")
        self.declare_parameter("sys_status_topic", "/mavros/sys_status")
        self.declare_parameter("downward_image_topic", "/downward_camera/image_jpeg")
        self.declare_parameter("d455_color_topic", "/camera/d455/color/image_raw")
        self.declare_parameter("d455_depth_topic", "/camera/d455/aligned_depth_to_color/image_raw")
        self.declare_parameter("d455_imu_topic", "/camera/d455/imu_filtered")
        self.declare_parameter("aruco_offset_topic", "/landing/aruco_offset")

        self.declare_parameter("image_sender_log", str(Path.home() / "isro_workspace/logs/pi_capture_log.csv"))
        self.declare_parameter("mission_log_path", "")
        self.declare_parameter("dashboard_json_topic", "/dashboard/pi_status_json")

        self.dashboard_url = str(self.get_parameter("dashboard_url").value)
        self.mac_receiver_health_url = str(self.get_parameter("mac_receiver_health_url").value)
        self.post_enabled = bool(self.get_parameter("post_enabled").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.post_timeout = float(self.get_parameter("post_timeout").value)
        self.stale_seconds = float(self.get_parameter("stale_seconds").value)
        self.mac_check_period_seconds = float(self.get_parameter("mac_check_period_seconds").value)

        self.vio_topic = str(self.get_parameter("vio_topic").value)
        self.local_pose_topic = str(self.get_parameter("local_pose_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.sys_status_topic = str(self.get_parameter("sys_status_topic").value)
        self.downward_image_topic = str(self.get_parameter("downward_image_topic").value)
        self.d455_color_topic = str(self.get_parameter("d455_color_topic").value)
        self.d455_depth_topic = str(self.get_parameter("d455_depth_topic").value)
        self.d455_imu_topic = str(self.get_parameter("d455_imu_topic").value)
        self.aruco_offset_topic = str(self.get_parameter("aruco_offset_topic").value)

        self.image_sender_log = Path(os.path.expanduser(str(self.get_parameter("image_sender_log").value)))
        self.mission_log_path = str(self.get_parameter("mission_log_path").value)
        self.dashboard_json_topic = str(self.get_parameter("dashboard_json_topic").value)

        self.hostname = socket.gethostname()

        self.vio_msg = None
        self.local_pose_msg = None
        self.state_msg = None
        self.sys_status_msg = None
        self.aruco_msg = None

        self.rates = {
            "vio": RateTracker(),
            "local_pose": RateTracker(),
            "state": RateTracker(),
            "sys_status": RateTracker(),
            "downward_image": RateTracker(),
            "d455_color": RateTracker(),
            "d455_depth": RateTracker(),
            "d455_imu": RateTracker(),
            "aruco": RateTracker()
        }

        self.last_mac_check_time = 0.0
        self.mac_receiver_status = {
            "reachable": False,
            "status_code": None,
            "age_sec": None,
            "error": "not_checked"
        }

        self.post_ok_count = 0
        self.post_fail_count = 0
        self.publish_count = 0

        self.json_pub = self.create_publisher(String, self.dashboard_json_topic, 10)

        self.create_subscription(Odometry, self.vio_topic, self.vio_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.local_pose_topic, self.local_pose_callback, qos_profile_sensor_data)
        self.create_subscription(State, self.state_topic, self.state_callback, qos_profile_sensor_data)
        self.create_subscription(SysStatus, self.sys_status_topic, self.sys_status_callback, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, self.downward_image_topic, self.downward_image_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.d455_color_topic, self.d455_color_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.d455_depth_topic, self.d455_depth_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, self.d455_imu_topic, self.d455_imu_callback, qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped, self.aruco_offset_topic, self.aruco_callback, qos_profile_sensor_data)

        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f"DASHBOARD_AGENT_READY dashboard_url={self.dashboard_url}")
        self.get_logger().info(f"JSON_TOPIC={self.dashboard_json_topic}")
        self.get_logger().info(f"POST_ENABLED={self.post_enabled}")

    def vio_callback(self, msg):
        self.vio_msg = msg
        self.rates["vio"].tick()

    def local_pose_callback(self, msg):
        self.local_pose_msg = msg
        self.rates["local_pose"].tick()

    def state_callback(self, msg):
        self.state_msg = msg
        self.rates["state"].tick()

    def sys_status_callback(self, msg):
        self.sys_status_msg = msg
        self.rates["sys_status"].tick()

    def downward_image_callback(self, msg):
        self.rates["downward_image"].tick()

    def d455_color_callback(self, msg):
        self.rates["d455_color"].tick()

    def d455_depth_callback(self, msg):
        self.rates["d455_depth"].tick()

    def d455_imu_callback(self, msg):
        self.rates["d455_imu"].tick()

    def aruco_callback(self, msg):
        self.aruco_msg = msg
        self.rates["aruco"].tick()

    def round_value(self, value, digits=3):
        if value is None:
            return None
        try:
            return round(float(value), digits)
        except Exception:
            return None

    def quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_pose_dict(self, msg):
        if msg is None:
            return None
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return {
            "x": self.round_value(p.x),
            "y": self.round_value(p.y),
            "z": self.round_value(p.z),
            "yaw": self.round_value(self.quat_to_yaw(q))
        }

    def pose_dict(self, msg):
        if msg is None:
            return None
        p = msg.pose.position
        q = msg.pose.orientation
        return {
            "x": self.round_value(p.x),
            "y": self.round_value(p.y),
            "z": self.round_value(p.z),
            "yaw": self.round_value(self.quat_to_yaw(q))
        }

    def odom_velocity_dict(self, msg):
        if msg is None:
            return None
        v = msg.twist.twist
        return {
            "linear_x": self.round_value(v.linear.x),
            "linear_y": self.round_value(v.linear.y),
            "linear_z": self.round_value(v.linear.z),
            "angular_x": self.round_value(v.angular.x),
            "angular_y": self.round_value(v.angular.y),
            "angular_z": self.round_value(v.angular.z)
        }

    def rate_block(self, key):
        tracker = self.rates[key]
        age = tracker.age()
        return {
            "alive": tracker.alive(self.stale_seconds),
            "rate_hz": self.round_value(tracker.hz(), 2),
            "age_sec": self.round_value(age, 2)
        }

    def parse_pose_from_filename(self, name):
        if not name:
            return None
        pattern = r"x(-?\d+(?:\.\d+)?)_y(-?\d+(?:\.\d+)?)_z(-?\d+(?:\.\d+)?)_yaw(-?\d+(?:\.\d+)?)"
        match = re.search(pattern, name)
        if not match:
            return None
        return {
            "x": self.round_value(match.group(1)),
            "y": self.round_value(match.group(2)),
            "z": self.round_value(match.group(3)),
            "yaw": self.round_value(match.group(4))
        }

    def read_image_sender_log(self):
        result = {
            "log_path": str(self.image_sender_log),
            "log_exists": self.image_sender_log.exists(),
            "total_rows": 0,
            "sent_count": 0,
            "failed_count": 0,
            "latest": None,
            "latest_pose_from_filename": None,
            "latest_age_sec": None
        }

        if not self.image_sender_log.exists():
            return result

        rows = []

        try:
            with self.image_sender_log.open("r", newline="") as file:
                reader = csv.DictReader(file)
                for row in reader:
                    rows.append(row)
        except Exception as error:
            result["error"] = str(error)
            return result

        result["total_rows"] = len(rows)

        if not rows:
            return result

        sent_count = 0
        failed_count = 0

        for row in rows:
            status = str(row.get("sent_status", ""))
            if "AUTO_SENT" in status:
                sent_count += 1
            if "FAILED" in status or "ERROR" in status:
                failed_count += 1

        latest = rows[-1]
        image_name = latest.get("image_name", "")

        result["sent_count"] = sent_count
        result["failed_count"] = failed_count
        result["latest"] = latest
        result["latest_pose_from_filename"] = self.parse_pose_from_filename(image_name)

        timestamp_text = latest.get("timestamp", "")

        try:
            dt = datetime.strptime(timestamp_text, "%Y-%m-%d %H:%M:%S")
            result["latest_age_sec"] = self.round_value(time.time() - dt.timestamp(), 2)
        except Exception:
            result["latest_age_sec"] = None

        return result

    def read_mission_log(self):
        path_text = str(self.mission_log_path).strip()
        result = {
            "log_path": path_text,
            "log_exists": False,
            "state": "UNKNOWN",
            "geofence_ok": True,
            "latest_event": None,
            "events": [],
            "error_count": 0,
            "warn_count": 0
        }

        if not path_text:
            result["state"] = "NO_LOG_PATH"
            return result

        path = Path(os.path.expanduser(path_text))
        result["log_path"] = str(path)
        result["log_exists"] = path.exists()

        if not path.exists():
            result["state"] = "LOG_NOT_FOUND"
            return result

        try:
            lines = path.read_text(errors="replace").splitlines()[-400:]
        except Exception as error:
            result["state"] = "LOG_READ_ERROR"
            result["error"] = str(error)
            return result

        keywords = [
            "HEALTH_PASS",
            "TAKEOFF",
            "PUBLISHING TAKEOFF SETPOINT",
            "HOVER",
            "LAWNMOWER_WAYPOINT_PLAN_CREATED",
            "MOVE_TO_XY",
            "LAWNMOWER_DONE_FULL_AREA_COVERED",
            "RTH_START",
            "RTH_REACHED_HOME",
            "SAFE_LAND_DISARM",
            "MISSION_DONE",
            "PHASE3_LAWNMOWER_DONE",
            "GEOFENCE_ABORT",
            "ERROR",
            "WARN"
        ]

        events = []

        for line in lines:
            if "ERROR" in line:
                result["error_count"] += 1
            if "WARN" in line:
                result["warn_count"] += 1
            if any(key in line for key in keywords):
                events.append(line[-300:])

        result["events"] = events[-25:]

        if result["events"]:
            result["latest_event"] = result["events"][-1]

        text = "\n".join(lines)

        if "GEOFENCE_ABORT" in text:
            result["state"] = "GEOFENCE_ABORT"
            result["geofence_ok"] = False
        elif "PHASE3_LAWNMOWER_DONE" in text or "MISSION_DONE" in text:
            result["state"] = "DONE"
        elif "SAFE_LAND_DISARM" in text:
            result["state"] = "LANDING_OR_DISARMED"
        elif "RTH_REACHED_HOME" in text:
            result["state"] = "RTH_REACHED_HOME"
        elif "RTH_START" in text:
            result["state"] = "RTH"
        elif "LAWNMOWER_DONE_FULL_AREA_COVERED" in text:
            result["state"] = "LAWNMOWER_DONE"
        elif "MOVE_TO_XY" in text or "LAWNMOWER_WAYPOINT_PLAN_CREATED" in text:
            result["state"] = "LAWNMOWER"
        elif "TAKEOFF" in text:
            result["state"] = "TAKEOFF"
        elif "HEALTH_PASS" in text:
            result["state"] = "HEALTH_READY"

        return result

    def mac_receiver_check(self):
        now = time.time()

        if now - self.last_mac_check_time < self.mac_check_period_seconds:
            self.mac_receiver_status["age_sec"] = self.round_value(now - self.last_mac_check_time, 2)
            return self.mac_receiver_status

        self.last_mac_check_time = now

        try:
            with urllib.request.urlopen(self.mac_receiver_health_url, timeout=self.post_timeout) as response:
                body = response.read().decode("utf-8", errors="replace")
                self.mac_receiver_status = {
                    "reachable": True,
                    "status_code": response.status,
                    "age_sec": 0.0,
                    "url": self.mac_receiver_health_url,
                    "body": body[:300],
                    "error": None
                }
        except Exception as error:
            self.mac_receiver_status = {
                "reachable": False,
                "status_code": None,
                "age_sec": 0.0,
                "url": self.mac_receiver_health_url,
                "body": None,
                "error": str(error)
            }

        return self.mac_receiver_status

    def battery_dict(self):
        if self.sys_status_msg is None:
            return {
                "available": False,
                "voltage_battery_raw": None,
                "voltage_battery_v": None,
                "current_battery": None,
                "battery_remaining": None
            }

        msg = self.sys_status_msg
        raw_voltage = getattr(msg, "voltage_battery", None)
        voltage_v = None

        if raw_voltage is not None:
            try:
                voltage_v = raw_voltage / 1000.0 if raw_voltage > 100 else raw_voltage
            except Exception:
                voltage_v = None

        return {
            "available": True,
            "voltage_battery_raw": raw_voltage,
            "voltage_battery_v": self.round_value(voltage_v, 2),
            "current_battery": getattr(msg, "current_battery", None),
            "battery_remaining": getattr(msg, "battery_remaining", None)
        }

    def flight_status(self):
        if self.state_msg is None:
            return "NO_STATE"

        if not self.state_msg.connected:
            return "DISCONNECTED"

        if self.state_msg.armed and self.state_msg.mode == "LAND":
            return "LANDING"

        if self.state_msg.armed:
            return "FLYING"

        if not self.state_msg.armed and self.state_msg.mode == "GUIDED":
            return "READY"

        return "DISARMED_OR_NOT_READY"

    def state_dict(self):
        if self.state_msg is None:
            return {
                "available": False,
                "connected": False,
                "armed": False,
                "guided": False,
                "manual_input": None,
                "mode": None,
                "system_status": None
            }

        return {
            "available": True,
            "connected": bool(self.state_msg.connected),
            "armed": bool(self.state_msg.armed),
            "guided": bool(self.state_msg.guided),
            "manual_input": bool(self.state_msg.manual_input),
            "mode": str(self.state_msg.mode),
            "system_status": int(self.state_msg.system_status)
        }

    def aruco_dict(self):
        if self.aruco_msg is None:
            return {
                "available": False,
                "alive": False,
                "offset_x": None,
                "offset_y": None,
                "distance": None,
                "age_sec": self.round_value(self.rates["aruco"].age(), 2)
            }

        return {
            "available": True,
            "alive": self.rates["aruco"].alive(self.stale_seconds),
            "offset_x": self.round_value(self.aruco_msg.vector.x),
            "offset_y": self.round_value(self.aruco_msg.vector.y),
            "distance": self.round_value(self.aruco_msg.vector.z),
            "age_sec": self.round_value(self.rates["aruco"].age(), 2)
        }

    def control_readiness(self):
        state = self.state_dict()
        local_ok = self.rates["local_pose"].alive(self.stale_seconds)
        vio_ok = self.rates["vio"].alive(self.stale_seconds)

        return {
            "display_only_recommended": True,
            "services": {
                "arming_service_ready": bool(self.arming_client.service_is_ready()),
                "set_mode_service_ready": bool(self.mode_client.service_is_ready()),
                "takeoff_service_ready": bool(self.takeoff_client.service_is_ready())
            },
            "can_send_land": bool(state["connected"] and self.mode_client.service_is_ready()),
            "can_send_disarm": bool(state["connected"] and self.arming_client.service_is_ready()),
            "can_start_mission": bool(
                state["connected"]
                and state["guided"]
                and not state["armed"]
                and local_ok
                and vio_ok
            ),
            "commands": {
                "emergency_land": "ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{base_mode: 0, custom_mode: 'LAND'}\"",
                "emergency_disarm": "ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: false}\""
            }
        }

    def build_payload(self):
        vio_block = self.rate_block("vio")
        local_block = self.rate_block("local_pose")
        state_block = self.rate_block("state")
        sys_block = self.rate_block("sys_status")

        state = self.state_dict()
        image_sender = self.read_image_sender_log()
        mission_log = self.read_mission_log()
        mac_receiver = self.mac_receiver_check()

        health = {
            "vio_fresh": bool(vio_block["alive"]),
            "local_position_fresh": bool(local_block["alive"]),
            "state_fresh": bool(state_block["alive"]),
            "mavros_connected": bool(state["connected"]),
            "guided": bool(state["guided"] and state["mode"] == "GUIDED"),
            "disarmed": bool(not state["armed"]),
            "phase2_health_pass_preflight": bool(
                vio_block["alive"]
                and local_block["alive"]
                and state_block["alive"]
                and state["connected"]
                and state["guided"]
                and state["mode"] == "GUIDED"
                and not state["armed"]
            ),
            "runtime_health_ok": bool(
                vio_block["alive"]
                and local_block["alive"]
                and state_block["alive"]
                and state["connected"]
                and state["guided"]
                and state["mode"] == "GUIDED"
            )
        }

        cameras = {
            "downward_camera": self.rate_block("downward_image"),
            "d455_color": self.rate_block("d455_color"),
            "d455_depth": self.rate_block("d455_depth"),
            "d455_imu_filtered": self.rate_block("d455_imu")
        }

        payload = {
            "schema": "isro_dashboard_agent_v1",
            "generated_at_unix": self.round_value(time.time(), 3),
            "agent": {
                "hostname": self.hostname,
                "node": "dashboard_agent",
                "publish_count": self.publish_count,
                "post_enabled": self.post_enabled,
                "dashboard_url": self.dashboard_url
            },
            "v1_monitoring": {
                "health": health,
                "vio": {
                    **vio_block,
                    "pose": self.odom_pose_dict(self.vio_msg),
                    "velocity": self.odom_velocity_dict(self.vio_msg),
                    "topic": self.vio_topic
                },
                "flight": {
                    "status": self.flight_status(),
                    "state": state,
                    "local_position": {
                        **local_block,
                        "pose": self.pose_dict(self.local_pose_msg),
                        "topic": self.local_pose_topic
                    },
                    "sys_status": {
                        **sys_block,
                        "battery": self.battery_dict(),
                        "topic": self.sys_status_topic
                    }
                },
                "cameras": cameras,
                "image_sender": image_sender,
                "aruco": self.aruco_dict(),
                "mac_link": {
                    "receiver": mac_receiver
                }
            },
            "v2_mission": {
                "mission_log": mission_log,
                "derived_state": mission_log.get("state", "UNKNOWN"),
                "geofence_ok": mission_log.get("geofence_ok", True),
                "flight_status": self.flight_status(),
                "current_pose": self.pose_dict(self.local_pose_msg),
                "latest_image_pose": image_sender.get("latest_pose_from_filename"),
                "latest_image": image_sender.get("latest", {}).get("image_name") if image_sender.get("latest") else None
            },
            "v3_control": self.control_readiness(),
            "mac_side_expected": {
                "receiver_folder": "~/isro_ws/test",
                "ref1_folder": "~/isro_ws/ref1_detections",
                "ref2_folder": "~/isro_ws/ref2_detections",
                "ref3_folder": "~/isro_ws/ref3_detections",
                "note": "Mac dashboard server should merge these Mac-side counts locally"
            }
        }

        return payload

    def post_payload(self, payload_text):
        if not self.post_enabled:
            return

        try:
            request = urllib.request.Request(
                self.dashboard_url,
                data=payload_text.encode("utf-8"),
                method="POST",
                headers={"Content-Type": "application/json"}
            )

            with urllib.request.urlopen(request, timeout=self.post_timeout) as response:
                response.read()

            self.post_ok_count += 1

            if self.post_ok_count == 1 or self.post_ok_count % 30 == 0:
                self.get_logger().info(f"DASHBOARD_POST_OK count={self.post_ok_count}")

        except Exception as error:
            self.post_fail_count += 1

            if self.post_fail_count == 1 or self.post_fail_count % 10 == 0:
                self.get_logger().warn(f"DASHBOARD_POST_FAILED count={self.post_fail_count} error={error}")

    def timer_callback(self):
        self.publish_count += 1
        payload = self.build_payload()
        payload["agent"]["post_ok_count"] = self.post_ok_count
        payload["agent"]["post_fail_count"] = self.post_fail_count

        payload_text = json.dumps(payload, separators=(",", ":"))

        msg = String()
        msg.data = payload_text
        self.json_pub.publish(msg)

        self.post_payload(payload_text)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
