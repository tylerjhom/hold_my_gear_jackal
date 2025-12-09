#!/usr/bin/env python3

print()"Starting Follow Mode")

import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


## Config
COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"
CMD_TOPIC   = "/j100_0000/cmd_vel"
MODEL_PATH  = "/home/robot/yolo11n.pt"

TARGET_DISTANCE = 1.5

MAX_SPEED = 0.7
MAX_TURN  = 1.2

RUN_RATE = 0.2

## Smoothing
VEL_ALPHA = 0.15 # smoothing filter strength
STOP_TIMEOUT = 0.8 # seconds without detection before stopping
DECAY = 0.85 # slowdown factor when target lost

EDGE_LOCK = 0.70
TURN_EXP  = 2.2


class YoloFollower(Node):

    def __init__(self):
        super().__init__("yolo_person_follower")
        self.model = YOLO(MODEL_PATH)
        self.bridge = CvBridge()

        self.latest_depth = None
        self.last_seen = time.time()
        self.last_cmd = np.array([0.0, 0.0])
        self.last_run = 0.0

        self.cmd_pub = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.create_subscription(Image, COLOR_TOPIC, self.image_cb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_cb, 10)

        self.get_logger().info("Follow Active")

    ## Depth
    def depth_cb(self, msg):
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.latest_depth = raw / 1000.0

    ## Vision
    def image_cb(self, msg):

        t = time.time()
        if t - self.last_run < RUN_RATE:
            return
        self.last_run = t

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model(frame, verbose=False)[0]

        closest = None
        best = 99

        for box in results.boxes:
            if int(box.cls[0]) != 0:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            if self.latest_depth is None:
                continue
            if cy >= self.latest_depth.shape[0] or cx >= self.latest_depth.shape[1]:
                continue

            d = float(self.latest_depth[cy, cx])
            if np.isnan(d) or d < 0.4:
                continue

            if d < best:
                best = d
                closest = cx, d, frame.shape[1]

        if closest:
            self.last_seen = time.time()
            self.track(closest)
        else:
            self.idle()


    ## Follow
    def track(self, person):

        cx, depth, width = person

        center_error = (cx - width/2) / (width/2)
        dist_error = depth - TARGET_DISTANCE

        # amplify turn when user approaches edge
        mag = abs(center_error)
        if mag > EDGE_LOCK:
            turn = np.sign(center_error) * MAX_TURN
        else:
            turn = np.sign(center_error) * ((mag ** TURN_EXP) * MAX_TURN)

        forward = np.clip(dist_error * 0.6, 0.0, MAX_SPEED)

        target = np.array([forward, -turn])

        # smooth transitions
        self.last_cmd = VEL_ALPHA * target + (1 - VEL_ALPHA) * self.last_cmd
        self.publish(self.last_cmd)

        self.get_logger().info(
            f"TRACK | d={depth:.2f} err={dist_error:.2f} "
            f"v={self.last_cmd[0]:.2f} w={self.last_cmd[1]:.2f}"
        )


    ## Lost target
    def idle(self):
        if time.time() - self.last_seen < STOP_TIMEOUT:
            self.last_cmd *= DECAY
        else:
            self.last_cmd[:] = 0

        self.publish(self.last_cmd)


    ## Publish
    def publish(self, cmd):

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x = float(cmd[0])
        msg.twist.angular.z = float(cmd[1])

        self.cmd_pub.publish(msg)


## Main
def main():

    rclpy.init()
    node = YoloFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.last_cmd[:] = 0
        node.publish(node.last_cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
