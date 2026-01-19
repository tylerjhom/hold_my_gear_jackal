# 02 - Nav2-Based Follower (YOLO → Pose Goal → Nav2)

This document explains how the **Nav2-based follower** works end-to-end on the Jackal. It focuses on how perception, goal generation, and navigation interact, and why this approach is preferred over direct velocity control for real-world operation.

---

## 1. Overview

In Nav2-based follow mode, the robot does **not** drive directly from perception outputs. Instead:

1. YOLO detects a firefighter in the camera image.
2. Depth data is used to estimate the firefighter’s position relative to the robot.
3. A relative follow goal is generated and published as a `PoseStamped`.
4. A Nav2 client node forwards that goal to the Nav2 stack.
5. Nav2 plans and executes a safe path, handling obstacle avoidance and velocity control.

This cleanly separates **perception and intent** from **navigation and control**.

---

## 2. Node Responsibilities

### 2.1 `yolo_nav2_follower.py` (Perception + Goal Generation)

This node is responsible for:
- Running YOLO on RGB images
- Selecting the firefighter to follow
- Estimating the firefighter’s relative position
- Publishing a follow goal on the `follow_goal` topic

It does **not** command the robot’s wheels directly.

---

### 2.2 `nav_to_pose_test.py` / `jackal_follow_nav_client` (Nav2 Interface)

This node:
- Subscribes to `follow_goal`
- Cancels any active Nav2 task when a new goal arrives
- Sends the new goal to Nav2 using `BasicNavigator`
- Monitors progress and cancels goals when appropriate

This node is the only component that directly interfaces with Nav2.

---

## 3. Topic and Data Flow

### 3.1 High-Level Topic Flow

```text
/camera/camera/color/image_raw         (Image)      \
/camera/camera/depth/image_rect_raw    (Image)       --> yolo_nav2_follower --> follow_goal (PoseStamped)
/camera/camera/color/camera_info       (CameraInfo) /

follow_goal (PoseStamped)
        |
        v
nav_to_pose_test / jackal_follow_nav_client
        |
        v
Nav2 (planner + controller)
        |
        v
/cmd_vel
````

---

## 4. Coordinate Frames

The YOLO node computes *relative* motion commands and publishes goals in the robot frame:

* `frame_id = base_link`
* `x` = forward displacement (meters)
* `y` = lateral displacement (meters)

This expresses the intent:

> “Move this far forward and sideways relative to the robot’s current pose.”

Nav2 ultimately plans and executes motion in the global `map` frame, so frame consistency must be handled correctly by the Nav2 client.

---

## 5. YOLO-Based Goal Computation

### 5.1 Target Selection

For each YOLO detection:

* Only COCO class `0` (person) is considered.
* The center pixel of the bounding box `(cx, cy)` is computed.
* Depth at that pixel is sampled from the depth image.
* Invalid samples are rejected:

  * NaN or infinite depth
  * Depth outside `[MIN_VALID_DEPTH, MAX_VALID_DEPTH]`
  * Pixel out of bounds

The closest valid person (minimum depth) is selected as the follow target.

---

### 5.2 Pixel to Metric Conversion

To compute lateral offset, the node uses the pinhole camera model:

```text
x_cam_right = (cx - cx_intrinsic) * depth / fx
```

Where:

* `cx_intrinsic` and `fx` come from `CameraInfo`
* `depth` is in meters

If `CameraInfo` is unavailable, a fallback focal length is estimated from a nominal horizontal field of view.

The lateral displacement used for navigation is then:

```text
lateral = -x_cam_right
```

A clamp is applied to prevent extreme lateral goals.

---

### 5.3 Forward Offset

Forward motion is computed as:

```text
forward = depth - TARGET_DISTANCE
```

This keeps the robot approximately `TARGET_DISTANCE` meters behind the firefighter.

---

## 6. Goal Filtering and Rate Limiting

Nav2 is not designed to accept continuously changing goals at sensor rates. To avoid excessive replanning, the YOLO node enforces:

### 6.1 Deadband

If both forward and lateral errors are small, no new goal is published.

---

### 6.2 Planar Delta Threshold

A new goal is only published if the change in position exceeds a minimum planar distance.

---

### 6.3 Cooldown Timer

Even if the goal changes, it will not be resent unless a minimum cooldown time has elapsed or the change is sufficiently large.

This prevents oscillations and goal churn.

---

## 7. Goal Message Format

The published message has the following structure:

* `header.frame_id = "base_link"`
* `pose.position.x = forward`
* `pose.position.y = lateral`
* Orientation set to identity (yaw is not commanded)

This represents a **relative waypoint**, not a final destination.

---

## 8. Nav2 Goal Execution

When a new follow goal is received:

1. Any active Nav2 task is canceled.
2. The new goal is forwarded to Nav2 using `goToPose()`.
3. Nav2:

   * Plans a collision-free path
   * Avoids obstacles using costmaps
   * Generates smooth velocity commands

The client node monitors feedback and cancels the goal if:

* The robot is within a short distance of the goal
* Navigation time exceeds a timeout

---

## 9. Frame Consistency Requirements

One of the most common failure modes is frame mismatch.

You must ensure **one** of the following is true:

* The YOLO node publishes goals in the `map` frame, or
* The Nav2 client transforms `base_link` goals into the `map` frame before calling `goToPose()`

If neither is done, Nav2 may interpret relative displacements as absolute map coordinates, leading to incorrect behavior.

---

## 10. Why Nav2-Based Following Is Recommended

Compared to direct velocity control, this approach provides:

* Obstacle avoidance via costmaps
* Stable, smooth motion
* Robust behavior in cluttered environments
* Reduced sensitivity to perception noise
* Clear separation of perception and control

This makes it suitable for real-world firefighter support scenarios.

---

## 11. Key Parameters to Tune

In `yolo_nav2_follower.py`:

* `TARGET_DISTANCE` – desired following distance
* `DEADBAND` – tolerance before ignoring updates
* `MIN_FORWARD_DELTA` – minimum positional change to trigger a new goal
* `GOAL_COOLDOWN_SEC` – minimum time between goals
* `MAX_LATERAL_GOAL` – lateral displacement clamp
* `RUN_RATE_SEC` – YOLO inference rate

---

## 12. Debugging Checklist

* Confirm YOLO detections are occurring
* Confirm `follow_goal` is being published:

  ```bash
  ros2 topic echo /follow_goal
  ```
* Confirm Nav2 is active and accepting goals
* Verify transforms between `map`, `odom`, and `base_link`
* Ensure frame IDs are consistent throughout the pipeline

---

## 13. Safety Notes

* Always test at low speed initially
* Maintain a joystick or E-stop override
* Supervise operation in dynamic environments
* Validate frame handling before autonomous use

```
```
