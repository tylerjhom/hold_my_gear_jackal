# 03 - Direct Velocity Follower (YOLO → /cmd_vel)

This document explains how the **Direct Velocity follower** works end-to-end. This mode bypasses Nav2 entirely and drives the robot by publishing velocity commands directly based on YOLO detections and depth measurements.

This approach is intentionally simple and is intended for testing, development, and controlled environments.

---

## 1. Overview

In Direct Velocity mode, the robot operates as a reactive visual servoing system:

1. YOLO detects a firefighter in the RGB image.
2. Depth data is used to estimate distance to the firefighter.
3. Horizontal image offset is converted into a turn command.
4. Distance error is converted into a forward velocity.
5. Commands are published directly to `/cmd_vel`.

There is no global planning, no obstacle avoidance, and no mapping.

---

## 2. Node Responsibilities

### 2.1 `yolo_follower.py`

This single node performs all functions:

- Subscribes to RGB and depth images
- Runs YOLO to detect a person
- Selects the closest valid target
- Computes linear and angular velocities
- Publishes velocity commands to the Jackal base

No Nav2 components are involved.

---

## 3. Topic and Data Flow

```text
/camera/camera/color/image_raw      (Image)  \
/camera/camera/depth/image_rect_raw (Image)   --> yolo_follower.py --> /cmd_vel
````

The Jackal base controller consumes `/cmd_vel` and drives the robot accordingly.

---

## 4. Target Selection

For each YOLO inference cycle:

1. All detections are filtered to COCO class `0` (person).
2. The bounding box center `(cx, cy)` is computed.
3. Depth at `(cx, cy)` is sampled.
4. Candidates are rejected if:

   * Depth is NaN or infinite
   * Depth is below a minimum threshold (sensor noise)
   * Pixel is out of bounds
5. The **closest valid person** (minimum depth) is selected as the target.

This ensures the robot follows the nearest firefighter when multiple people are present.

---

## 5. Control Law

The follower computes two independent control signals:

* Angular velocity (turning)
* Linear velocity (forward motion)

These are derived from image-space and depth errors.

---

### 5.1 Horizontal Error → Angular Velocity

The horizontal error is normalized by image width:

```text
center_error = (cx - image_width / 2) / (image_width / 2)
```

This produces a value approximately in the range `[-1, +1]`.

Turning behavior uses a nonlinear mapping:

* If the target is near the image edge, the robot turns at maximum rate.
* Near the image center, turning is gentle to reduce jitter.

Key parameters:

* `EDGE_LOCK` – threshold for full-rate turning
* `TURN_EXP` – exponent shaping the response curve
* `MAX_TURN` – maximum angular velocity

This design yields stable tracking near center while still reacting aggressively when the target approaches the edge of the camera view.

---

### 5.2 Distance Error → Forward Velocity

Forward motion is computed from distance error:

```text
dist_error = depth - TARGET_DISTANCE
linear_x = clip(dist_error * k, 0, MAX_SPEED)
```

Behavior:

* If the firefighter is farther than the target distance, the robot moves forward.
* If the firefighter is closer than the target distance, forward velocity is clamped to zero.
* The robot does not reverse in this mode.

This prevents oscillations and unintended backward motion.

---

## 6. Motion Smoothing

Raw velocity commands are filtered using an exponential moving average:

```text
cmd = alpha * new_cmd + (1 - alpha) * previous_cmd
```

Where:

* `VEL_ALPHA` controls responsiveness vs smoothness

This smoothing:

* Reduces jerk
* Mitigates noise from YOLO and depth sampling
* Produces more stable motion on the real robot

---

## 7. Lost-Target Handling

The follower does not immediately stop if the firefighter is momentarily lost.

Instead:

1. If the target was seen recently:

   * Velocity commands decay gradually using a multiplicative factor.
2. If the target has been lost longer than a timeout:

   * Commands are set to zero.

Parameters:

* `STOP_TIMEOUT` – duration before full stop
* `DECAY` – velocity decay factor

This allows brief perception dropouts without abrupt stops.

---

## 8. Command Publication

Velocity commands are published as:

* Linear velocity: `twist.linear.x`
* Angular velocity: `twist.angular.z`

Depending on configuration, the message type may be:

* `geometry_msgs/Twist`, or
* `geometry_msgs/TwistStamped`

The base controller must be configured to accept the chosen message type.

---

## 9. What This Mode Does Not Do

The Direct Velocity follower does **not** provide:

* Obstacle avoidance
* Global or local planning
* Recovery behaviors
* Map awareness
* Crowd disambiguation

The robot will drive into obstacles if the firefighter moves behind them.

---

## 10. When to Use This Mode

This mode is appropriate for:

* Open, uncluttered environments
* Early perception and control testing
* Demonstrations at very low speed
* Debugging YOLO and depth alignment

It is *not recommended* for operational deployment in cluttered or dynamic environments.

---

## 11. Key Parameters to Tune

In `yolo_follower.py`:

* `TARGET_DISTANCE` – desired following distance
* `MAX_SPEED` – maximum forward speed
* `MAX_TURN` – maximum angular velocity
* `EDGE_LOCK` – screen edge turn threshold
* `TURN_EXP` – turning response curve exponent
* `VEL_ALPHA` – command smoothing factor
* `STOP_TIMEOUT` – lost-target timeout
* `DECAY` – velocity decay rate
* `RUN_RATE` – YOLO inference interval

---

## 12. Debugging Checklist

* Verify YOLO detections are occurring
* Verify depth is aligned with RGB image
* Confirm `/cmd_vel` is being published:

  ```bash
  ros2 topic echo /j100_0000/cmd_vel
  ```
* Ensure the base controller accepts the command message type
* Reduce speed limits during initial testing

---

## 13. Safety Notes

* Always test with low speed limits
* Maintain a joystick or E-stop override
* Do not operate in crowded or cluttered environments
* Supervise at all times during operation

```
```
