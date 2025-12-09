#!/usr/bin/env python3

print("Starting YOLO to Nav2 Follower")

import time

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

from ultralytics import YOLO


# Robot Config Stuff
FOLLOW_GOAL_TOPIC = "follow_goal"

COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"
COLOR_INFO_TOPIC = "/camera/camera/color/camera_info"

MODEL_PATH = "/home/robot/yolo11n.pt" # make sure this exists

TARGET_DISTANCE = 1.5 # meters we want to stay from the person
RUN_RATE_SEC = 1.0 # YOLO inference rate

MIN_VALID_DEPTH = 0.4 # ignore anything closer than this (sensor noise)
MAX_VALID_DEPTH = 8.0 # ignore anything beyond this
DEADBAND = 0.4

# only send a new goal if the forward distance changes by this much
MIN_FORWARD_DELTA = 0.5 # meters (now used as planar delta threshold)

GOAL_COOLDOWN_SEC = 3.0 # To not send goals immediately one after the other 

# If CameraInfo is missing, fall back to a rough HFOV estimate to compute fx
DEFAULT_H_FOV_DEG = 69.0

# Prevent crazy lateral commands when the person is at screen edge / far away
MAX_LATERAL_GOAL = 2.0 # meters


class YoloNav2Follower(Node):
    def __init__(self):
        super().__init__("yolo_nav2_follower")

        self.get_logger().info("Node constructor entered.")

        self.bridge = CvBridge()

        self.get_logger().info(f"Loading YOLO model from: {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info("YOLO model loaded successfully.")

        self.latest_depth = None
        self.last_inference_time = 0.0

        self.last_goal_forward = None
        self.last_goal_lateral = None
        self.last_goal_time = 0.0

        # Camera intrinsics 
        self.cam_fx = None
        self.cam_cx = None
        self._warned_no_caminfo = False

        self.goal_pub = self.create_publisher(PoseStamped, FOLLOW_GOAL_TOPIC, 10)
        self.get_logger().info(f"Publishing Nav2 follow goals on: {FOLLOW_GOAL_TOPIC}")

        self.create_subscription(Image, COLOR_TOPIC, self.image_cb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_cb, 10)
        self.create_subscription(CameraInfo, COLOR_INFO_TOPIC, self.caminfo_cb, 10)

        self.depth_frames_seen = 0
        self.image_frames_seen = 0

        self.get_logger().info(
            f"Subscribed to color: {COLOR_TOPIC} and depth: {DEPTH_TOPIC}"
        )
        self.get_logger().info(f"Subscribed to camera info: {COLOR_INFO_TOPIC}")
        self.get_logger().info("YOLO → Nav2 follower ready.")


    # Callbacks!
    def caminfo_cb(self, msg: CameraInfo):

        if self.cam_fx is None:
            try:
                self.cam_fx = float(msg.k[0])
                self.cam_cx = float(msg.k[2])
                self.get_logger().info(
                    f"[CAMINFO] Received intrinsics: fx={self.cam_fx:.2f}, cx={self.cam_cx:.2f}"
                )
            except Exception as e:
                self.get_logger().warn(f"[CAMINFO] Failed to parse CameraInfo K matrix: {e}")

    def depth_cb(self, msg: Image):
        self.depth_frames_seen += 1
        if self.depth_frames_seen % 30 == 1:
            self.get_logger().info(
                f"[DEPTH_CB] Received depth frame #{self.depth_frames_seen}"
            )

        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.latest_depth = raw.astype(np.float32) / 1000.0  # mm → meters

    def image_cb(self, msg: Image):
        self.image_frames_seen += 1
        now = time.time()

        # throttle YOLO rate
        if now - self.last_inference_time < RUN_RATE_SEC:
            if self.image_frames_seen % 30 == 1:
                self.get_logger().debug(
                    "Skipping frame due to rate limit."
                )
            return

        self.last_inference_time = now

        if self.latest_depth is None:
            self.get_logger().warn("[IMAGE_CB] No depth yet; skipping.")
            return

        self.get_logger().info(
            f"[IMAGE_CB] Running YOLO on frame #{self.image_frames_seen}"
        )

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Yolo Reference
        results = self.model(frame, verbose=False)[0]
        detections = results.boxes
        self.get_logger().info(
            f"[YOLO] Total detections: {len(detections)}"
        )

        closest_person = None
        min_distance = float("inf")

        # Bounding Boxes
        for idx, box in enumerate(detections):
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            self.get_logger().info(
                f"[YOLO] Det {idx}: cls={cls}, conf={conf:.2f}, "
                f"bbox=({x1},{y1})-({x2},{y2}), center=({cx},{cy})"
            )

            if cls != 0:  # 0 = person in COCO
                continue

            # bounds check
            if not (0 <= cy < self.latest_depth.shape[0]) or not (0 <= cx < self.latest_depth.shape[1]):
                self.get_logger().warn(
                    f"[DEPTH] Center ({cx},{cy}) out of depth image bounds "
                    f"{self.latest_depth.shape}"
                )
                continue

            depth_val = float(self.latest_depth[cy, cx])

            self.get_logger().info(
                f"[DEPTH] Person candidate at ({cx},{cy}) depth={depth_val:.2f} m"
            )

            if not np.isfinite(depth_val):
                self.get_logger().warn("[DEPTH] Depth is NaN/inf; skipping candidate.")
                continue

            if depth_val < MIN_VALID_DEPTH or depth_val > MAX_VALID_DEPTH:
                self.get_logger().info(
                    f"[DEPTH] Depth {depth_val:.2f} out of valid range "
                    f"[{MIN_VALID_DEPTH}, {MAX_VALID_DEPTH}]; skipping."
                )
                continue

            if depth_val < min_distance:
                min_distance = depth_val
                closest_person = (cx, cy, depth_val)

        if closest_person:
            self.get_logger().info(
                f"[FOLLOW] Closest person: pixel=({closest_person[0]},"
                f"{closest_person[1]}), depth={closest_person[2]:.2f} m"
            )
            self.send_follow_goal(closest_person)
        else:
            self.get_logger().info("[FOLLOW] No valid person found in this frame.")


    # Goal Logic
    def send_follow_goal(self, person):
        cx, cy, depth = person
        now = time.time()


        if (self.cam_fx is not None) and (self.cam_cx is not None):
            x_cam_right = (float(cx) - self.cam_cx) * depth / self.cam_fx
        else:
            # fallback if CameraInfo not available
            if not self._warned_no_caminfo:
                self.get_logger().warn(
                    "[CAMINFO] No CameraInfo intrinsics yet; using HFOV fallback estimate."
                )
                self._warned_no_caminfo = True

            w = float(self.latest_depth.shape[1])
            fx_guess = (w / 2.0) / np.tan(np.deg2rad(DEFAULT_H_FOV_DEG) / 2.0)
            x_cam_right = (float(cx) - (w / 2.0)) * depth / fx_guess

        lateral = -x_cam_right
        lateral = float(np.clip(lateral, -MAX_LATERAL_GOAL, MAX_LATERAL_GOAL))

        forward = depth - TARGET_DISTANCE

        self.get_logger().info(
            f"[GOAL] depth={depth:.2f} m, TARGET={TARGET_DISTANCE:.2f} → "
            f"forward={forward:.2f} m, lateral={lateral:.2f} m"
        )

        # Deadband: already close enough (considers forward + lateral)
        if abs(forward) < DEADBAND and abs(lateral) < DEADBAND:
            self.get_logger().info(
                f"[GOAL] forward={forward:.2f}, lateral={lateral:.2f} within deadband={DEADBAND:.2f}; "
                "not sending new goal."
            )
            return

        # If we've sent a goal recently and it's not a big change, skip
        if self.last_goal_forward is not None and self.last_goal_lateral is not None:
            dx = forward - self.last_goal_forward
            dy = lateral - self.last_goal_lateral
            delta = float(np.hypot(dx, dy))
            time_since = now - self.last_goal_time

            self.get_logger().info(
                f"[GOAL] last(x,y)=({self.last_goal_forward:.2f},{self.last_goal_lateral:.2f}) → "
                f"delta={delta:.2f} (min {MIN_FORWARD_DELTA:.2f}), "
                f"time_since_last={time_since:.2f}s"
            )

            if delta < MIN_FORWARD_DELTA and time_since < GOAL_COOLDOWN_SEC:
                self.get_logger().info(
                    "[GOAL] Change too small AND cooldown not elapsed; "
                    "skipping goal to avoid Nav2 churn."
                )
                return

        # Create new goal
        goal = PoseStamped()
        goal.header.frame_id = "base_link" # Switched everything to base_link to give relative to robot reference frame
        #  goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = forward
        goal.pose.position.y = lateral
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.last_goal_forward = forward
        self.last_goal_lateral = lateral
        self.last_goal_time = now

        self.get_logger().info(
            f"[GOAL] Published follow goal: x={forward:.2f}, y={lateral:.2f} "
            f"(depth={depth:.2f} m, px=({cx},{cy}))"
        )


def main():
    rclpy.init()
    node = YoloNav2Follower()
    node.get_logger().info("Waiting for Nav2 + YOLO frames...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt; shutting down YOLO to Nav2 follower.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
