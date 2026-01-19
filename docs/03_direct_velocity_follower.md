---

## `docs/03_offboard_setup.md`

```md
# 03 — Direct Velocity Follower (YOLO → /cmd_vel)

This document explains how the **Direct Velocity follower** works end-to-end, including:
- what inputs it uses
- how it computes turn + forward velocity
- smoothing and lost-target behavior
- why it is simpler but less safe than Nav2

> **Summary:** YOLO detects a person and publishes `/cmd_vel` directly to keep the person centered and ~1.5 m away. There is **no planning** and **no obstacle avoidance**.

---

## 1) The Direct Velocity Architecture

### 1.1 Node Roles

This mode typically uses a single node:

- **`yolo_follower.py`**  
  *Perception + control*  
  - subscribes to RGB + depth  
  - runs YOLO to detect a person  
  - estimates distance and horizontal offset  
  - computes linear + angular velocity commands  
  - publishes `/cmd_vel` (TwistStamped in your implementation)

There is **no Nav2** in the control loop.

---

## 2) Topic Data Flow

```text
/camera/camera/color/image_raw      (Image)  \
/camera/camera/depth/image_rect_raw (Image)   --> yolo_follower.py --> /cmd_vel (TwistStamped)
````

The base controller drives the Jackal from `/cmd_vel`.

> Note: Some controllers expect `geometry_msgs/Twist` not `TwistStamped`. Ensure your Clearpath config matches.

---

## 3) What `yolo_follower.py` Does (Step-by-Step)

### 3.1 Throttled YOLO Loop

The callback runs YOLO no faster than:

* `RUN_RATE = 0.2` seconds → ~5 Hz

This prevents high CPU usage and reduces command jitter.

---

### 3.2 Person Detection and Target Selection

The node:

* runs YOLO on the RGB frame
* filters detections to class 0 (person)
* for each person:

  * compute bbox center `(cx, cy)`
  * read depth at that pixel
  * reject invalid depth:

    * NaN
    * too close (< 0.4 m)
* choose the closest valid person (minimum depth)

So the target is **the closest detected person**, not “largest bbox”.

---

## 4) The Control Law (How it decides v and w)

The node computes two error signals:

### 4.1 Horizontal centering error → angular velocity

```text
center_error = (cx - image_width/2) / (image_width/2)
```

This normalizes pixel offset into approximately [-1, +1].

Then it computes turn rate with a nonlinear “edge lock” behavior:

* if the person is near the edge, turn at max
* otherwise, use a power curve to reduce jitter near center

Parameters:

* `EDGE_LOCK = 0.70`
* `TURN_EXP = 2.2`
* `MAX_TURN = 1.2 rad/s`

This yields:

* **gentle turning** when close to center
* **aggressive turning** when the person approaches the edge

---

### 4.2 Distance error → forward velocity

```text
dist_error = depth - TARGET_DISTANCE
forward = clip(dist_error * 0.6, 0, MAX_SPEED)
```

Key behavior:

* If person is far → forward is positive → move forward
* If person is too close → forward would be negative, but it is clipped to 0
  → robot **will not reverse** in this mode

Parameter:

* `TARGET_DISTANCE = 1.5 m`
* `MAX_SPEED = 0.7 m/s`

---

### 4.3 Command sign convention

The node publishes:

```text
cmd = [forward, -turn]
```

That negative sign is chosen to match your camera/robot yaw convention.

If the robot turns away from the person, flip the sign.

---

## 5) Motion Smoothing (Why it doesn’t “snap” each frame)

The node applies a low-pass filter to [v, w]:

```text
last_cmd = alpha * target_cmd + (1 - alpha) * last_cmd
```

Where:

* `VEL_ALPHA = 0.15`

Interpretation:

* only 15% of the new command is applied each update
* 85% comes from the previous command

This reduces jerk and makes following feel “smooth”.

---

## 6) Lost Target Behavior (Decay then Stop)

If the person is not detected, it does **not** immediately stop.

It uses:

* `STOP_TIMEOUT = 0.8 s`
* `DECAY = 0.85`

Logic:

1. If the person was seen recently (< timeout):

   * multiply commands by DECAY (ramps down)
2. If lost longer than timeout:

   * set commands to zero

This helps when YOLO briefly misses a frame.

---

## 7) What This Mode Does NOT Do

* ❌ obstacle avoidance
* ❌ global planning
* ❌ recovery behaviors
* ❌ stable tracking in crowds
* ❌ map-based navigation

This mode will drive into obstacles if the person is behind them.

---

## 8) When to Use This Mode

Use the direct velocity follower for:

* open areas
* controlled environments
* quick perception/controller prototyping
* verifying�RW¼Zr8`JݢNvWă\N5\f

Do NOT use it for cluttered indoor spaces unless a human has an immediate stop override.

---

## 9) Tuning Guide (What to Change First)

* Follow distance:

  * `TARGET_DISTANCE`
* Smoothness:

  * `VEL_ALPHA` (higher = snappier, lower = smoother)
* Turn aggressiveness:

  * `MAX_TURN`
  * `TURN_EXP`
  * `EDGE_LOCK`
* Maximum approach speed:

  * `MAX_SPEED`
* Lost-target behavior:

  * `STOP_TIMEOUT`
  * `DECAY`

---

## 10) Debugging Tips

### Verify commands are publishing

```bash
ros2 topic echo /j100_0000/cmd_vel
```

### Verify base is accepting commands

* confirm controller listens to TwistStamped if used
* check Clearpath topics + mux settings

### Verify depth is aligned

If depth isn’t aligned with RGB, the depth sampled at (cx,cy) will be wrong and behavior becomes unstable.

---

## 11) Safety Notes

* Start with low MAX_SPEED
* Keep an E-stop or joystick override active
* Test with clear space and soft barriers first
