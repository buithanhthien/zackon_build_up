# Rear-Lidar-Only Docking Implementation Summary

## Overview
This document explains the modifications made to implement a simpler rear-lidar-only docking strategy with near-range stopping for ROS 2 Jazzy.

---

## PHASE A — REAR-FACING STAGING POSE CONFIGURATION

### Problem
The original configuration had the robot arriving at the staging pose **facing forward** toward the dock:
- `staging_yaw_offset: 0.0` → robot faces dock
- Front lidar would see the reflective tapes
- Controller would reverse into dock

But you wanted the robot to arrive with its **REAR facing the dock** so the rear lidar can detect tapes from the start.

### Solution
**Changed `staging_yaw_offset` from `0.0` to `π` (3.14159 rad)**

**Why this works:**
- `staging_x_offset: -0.80` keeps the robot 0.8m **behind** the dock (unchanged)
- `staging_yaw_offset: π` rotates the staging orientation by 180°
- Robot now arrives at staging with its **rear facing the dock**
- Rear lidar immediately sees the reflective tapes
- Controller drives **forward** (not backward) into the dock

**What NOT to change:**
- **Dock pose yaw** should NOT be changed — it defines the dock's orientation in the map frame
- **staging_x_offset** should NOT be changed — it already positions the robot behind the dock

**Related parameter changes:**
- `dock_direction: "forward"` (was "backward") — controller now drives forward
- `rotate_to_dock: false` (was true) — no extra rotation needed

---

## PHASE B — REAR LIDAR TOPIC AND THRESHOLD TUNING

### Changes Made

1. **Scan topic changed:**
   ```yaml
   # OLD: scan_topic: "/scan_front_filter"
   scan_topic: "/scan_rear_lidar_filter"  # NEW: rear lidar
   ```

2. **LRF offset changed (now negative for rear lidar):**
   ```yaml
   # OLD: lrf_forward_offset: 0.35   # Front lidar (positive = forward)
   lrf_forward_offset: -0.35          # NEW: Rear lidar (negative = backward)
   ```

3. **Intensity thresholds tuned for rear lidar:**
   Based on your observed rear lidar statistics (peak-valley pair: 46-21):
   ```yaml
   # OLD: i_peak: 43.0, i_valley: 29.0  # Front lidar thresholds
   i_peak: 46.0                          # NEW: Rear lidar peak
   i_valley: 21.0                        # NEW: Rear lidar valley
   ```

### Why These Values
Your observation at staging showed:
- Rear lidar reflective peak: ~46
- Rear lidar valley: ~21

These thresholds are tuned to match the rear lidar's reflective characteristics, which differ from the front lidar due to mounting position, angle, and environmental factors.

---

## PHASE C — NEAR-RANGE STOPPING LOGIC

### The Root Problem

**Why the old `isDocked()` fails near the dock:**

The original implementation computes distance as:
```cpp
double dx = dock_in_odom.pose.position.x - robot_tf.transform.translation.x;
double dy = dock_in_odom.pose.position.y - robot_tf.transform.translation.y;
double dist = std::hypot(dx, dy);
```

Where:
- `dx`, `dy` = 2D offset from robot's current position to the **last detected dock pose**
- Both positions are in the **odom frame** (fixed odometry frame)
- `dist` = 2D Euclidean distance between robot and dock

**The problem occurs when:**
1. Robot gets very close to the dock
2. The 2-reflector pair becomes unstable or disappears (beams too close, geometry breaks down)
3. `detectReflectors()` fails → `dock_detected_` becomes false after `max_fail_count_` misses
4. Code falls back to `refined_pose_latched_` (last good pose before detection was lost)
5. **But `refined_pose_latched_` is FROZEN** — it doesn't update as the robot continues moving
6. So `dist` becomes **STALE** and may never drop below `docking_threshold_`
7. **Result:** Docking fails with timeout error 905

### The Solution: Two-Mode Docking Detection

The new `isDocked()` implementation supports two modes:

#### **MODE 1 — REFLECTIVE-POSE MODE** (medium range, reflectors visible)
- Used when the 2-reflector pair is still detected reliably
- Uses the current pose-based distance logic:
  - If `dock_detected_ == true`: use `last_detected_pose_` (continuously updated)
  - If `dock_detected_ == false` but `has_refined_pose_latch_ == true`: use `refined_pose_latched_`
  - Transform to odom, compute `dist = hypot(dx, dy)`
  - Return `true` if `dist < docking_threshold_` (0.32m default)

#### **MODE 2 — NEAR-RANGE DIRECT-RANGE MODE** (very close, reflectors unstable)
- Activates when `dist < near_range_entry_distance_` (0.50m default)
- Uses **direct range measurement** from the rear lidar scan
- Process:
  1. Extract ranges from a narrow angular sector around the rear direction
     - Sector width: ±`near_range_sector_half_angle_deg_` (default ±15°)
     - Sector center: 0 rad in rear lidar frame (pointing backward)
  2. Compute robust range estimate using configured statistic:
     - **"min"**: minimum valid range (most conservative, sensitive to noise)
     - **"median"**: median of valid ranges (robust to outliers, **recommended**)
     - **"trimmed_mean"**: mean after removing top/bottom 10% outliers
  3. If `range < near_range_stop_threshold_` (0.10m default):
     - Increment `near_range_stable_count_`
     - If `count >= near_range_required_stable_count_` (3 cycles) → return `true` (DOCKED)
  4. Else: reset counter

### New Parameters Added

```yaml
# Enable/disable near-range mode
use_near_range_stop: true

# Distance below which near-range mode can activate
near_range_entry_distance: 0.50  # meters

# Angular sector width for range sampling (±degrees from rear direction)
near_range_sector_half_angle_deg: 15.0

# Range threshold for declaring docked in near-range mode
near_range_stop_threshold: 0.10  # meters

# Debounce: consecutive cycles below threshold required
near_range_required_stable_count: 3

# Statistic for robust range estimate: "min", "median", "trimmed_mean"
near_range_statistic: "median"
```

### Why This Works

**Advantages of the two-mode approach:**
1. **Medium range (0.5m - 3.0m):** Uses reflective detection for accurate pose estimation
2. **Close range (< 0.5m):** Switches to direct range measurement, robust to reflector loss
3. **Debouncing:** Requires 3 consecutive cycles below threshold to avoid false positives from noise
4. **Robust statistics:** Median filtering removes outliers and noise spikes
5. **No stale data:** Direct range measurement always uses the latest scan

---

## Code Structure Changes

### Files Modified

1. **`docking_params.yaml`**
   - Changed scan topic to rear lidar
   - Adjusted intensity thresholds for rear lidar
   - Changed staging_yaw_offset to π for rear-facing staging
   - Added 6 new near-range parameters

2. **`lidar_intensity_dock.hpp`**
   - Added 6 new parameter member variables
   - Added `last_scan_` to store latest scan
   - Added `near_range_stable_count_` for debouncing
   - Added `scan_mutex_` to protect scan access
   - Added `dock_near_range_pub_` for debug output
   - Added `computeNearRangeEstimate()` helper method

3. **`lidar_intensity_dock.cpp`**
   - Updated `configure()` to load new parameters
   - Updated `activate()` to reset near-range state and create debug publisher
   - Updated `deactivate()` to cleanup new publisher
   - Updated `scanCallback()` to store latest scan
   - **Completely rewrote `isDocked()`** with two-mode logic
   - Added `computeNearRangeEstimate()` implementation (130 lines)

### New Debug Topic

**`/dock_near_range`** (std_msgs/Float32)
- Published when near-range mode is active
- Shows the computed robust range estimate
- Useful for tuning `near_range_stop_threshold_`

---

## Testing and Tuning Guide

### Step 1: Verify Rear Lidar Detection at Staging
1. Navigate robot to staging pose
2. Check `/detected_dock_pose` topic — should see stable detections
3. If not detecting:
   - Adjust `i_peak` and `i_valley` based on `ros2 topic echo /scan_rear_lidar_filter`
   - Look for intensity spikes on the reflective tapes

### Step 2: Monitor Reflective-Pose Mode
1. Start docking approach
2. Watch `/dock_distance` topic
3. Should see distance decreasing smoothly until ~0.5m

### Step 3: Monitor Near-Range Mode Activation
1. When distance drops below 0.5m, near-range mode activates
2. Watch `/dock_near_range` topic
3. Should see range decreasing as robot approaches
4. When range < 0.10m for 3 cycles, docking completes

### Step 4: Tune Parameters if Needed

**If docking stops too early:**
- Decrease `near_range_stop_threshold` (e.g., 0.08m)

**If docking doesn't stop (overshoots):**
- Increase `near_range_stop_threshold` (e.g., 0.12m)
- Increase `near_range_required_stable_count` (e.g., 5)

**If near-range estimate is noisy:**
- Increase `near_range_sector_half_angle_deg` (e.g., 20°) for more samples
- Change `near_range_statistic` to "trimmed_mean" for better outlier rejection

**If near-range mode activates too late:**
- Increase `near_range_entry_distance` (e.g., 0.60m)

---

## Important Notes

### Lidar Frame Convention
The `computeNearRangeEstimate()` method assumes:
- Rear lidar's 0° direction points **directly backward** (toward the dock)
- Sector center is at 0 rad in the lidar's frame

**If your rear lidar is mounted differently:**
- Adjust `sector_center` in `computeNearRangeEstimate()` accordingly
- Example: If lidar is rotated 180°, use `sector_center = M_PI`

### Coordinate Frames Summary
- **map frame**: Global map frame (dock pose is defined here)
- **odom frame**: Fixed odometry frame (used for smooth control, no map jumps)
- **base_link frame**: Robot's base frame (origin at robot center)
- **rear_lidar frame**: Rear lidar's optical frame (origin at lidar sensor)

### Debugging Commands

```bash
# Monitor dock distance (reflective-pose mode)
ros2 topic echo /dock_distance

# Monitor near-range estimate (near-range mode)
ros2 topic echo /dock_near_range

# Monitor detected dock pose
ros2 topic echo /detected_dock_pose

# Monitor dock pose in odom frame
ros2 topic echo /dock_pose_in_odom

# Check rear lidar intensities
ros2 topic echo /scan_rear_lidar_filter
```

---

## Summary of Key Design Decisions

1. **Why change `staging_yaw_offset` instead of dock pose yaw?**
   - Dock pose defines the dock's orientation in the map (should not change)
   - Staging pose is relative to the dock (safe to modify)
   - Changing staging_yaw_offset is the cleanest way to flip robot orientation

2. **Why use median instead of min for near-range estimate?**
   - Min is too sensitive to noise and outliers
   - Median is robust and filters out spurious readings
   - Trimmed mean is also good but more complex

3. **Why require 3 consecutive cycles below threshold?**
   - Prevents false positives from momentary noise spikes
   - Ensures robot has truly reached the dock
   - 3 cycles ≈ 60ms at 50Hz controller frequency (fast enough)

4. **Why use a ±15° sector instead of full scan?**
   - Narrow sector focuses on the dock directly ahead
   - Avoids contamination from side obstacles
   - Still wide enough to capture sufficient samples for robust statistics

5. **Why activate near-range mode at 0.5m instead of immediately?**
   - Reflective detection is more accurate at medium range (better pose estimation)
   - Near-range mode is a fallback for when reflectors become unreliable
   - 0.5m is a good transition point where reflectors start to become unstable

---

## Potential Issues and Solutions

### Issue 1: Robot doesn't detect dock at staging
**Symptoms:** No `/detected_dock_pose` messages, docking fails with error 904
**Solutions:**
- Check rear lidar topic: `ros2 topic echo /scan_rear_lidar_filter`
- Verify intensity values on reflective tapes
- Adjust `i_peak` and `i_valley` thresholds
- Check `lrf_forward_offset` sign (should be negative for rear lidar)

### Issue 2: Docking stops too far from dock
**Symptoms:** isDocked() returns true but robot is 15-20cm away
**Solutions:**
- Decrease `near_range_stop_threshold` (e.g., 0.08m)
- Check `/dock_near_range` to see actual measured range
- Verify rear lidar is not tilted (check `lrf_tilt_alpha_deg`)

### Issue 3: Docking overshoots and collides
**Symptoms:** Robot drives past the dock
**Solutions:**
- Increase `near_range_stop_threshold` (e.g., 0.12m)
- Increase `near_range_required_stable_count` (e.g., 5)
- Check controller velocity limits in `docking_params.yaml`

### Issue 4: Near-range mode never activates
**Symptoms:** Docking fails with timeout, `/dock_near_range` never publishes
**Solutions:**
- Check `use_near_range_stop: true` in config
- Verify `near_range_entry_distance` is larger than `docking_threshold`
- Check logs for TF errors or scan availability

### Issue 5: Near-range estimate is unstable/noisy
**Symptoms:** `/dock_near_range` jumps around, docking is jerky
**Solutions:**
- Change `near_range_statistic` to "trimmed_mean"
- Increase `near_range_sector_half_angle_deg` for more samples
- Increase `near_range_required_stable_count` for more filtering
- Check for obstacles in the rear sector

---

## Next Steps

1. **Build the package:**
   ```bash
   cd ~/zackon_build_up
   colcon build --packages-select lidar_dock_detector
   source install/setup.bash
   ```

2. **Test at staging pose:**
   - Manually drive robot to staging pose
   - Verify rear lidar detects reflective tapes
   - Check `/detected_dock_pose` is stable

3. **Test full docking sequence:**
   - Run docking action
   - Monitor all debug topics
   - Verify smooth transition from reflective-pose to near-range mode

4. **Tune parameters:**
   - Adjust thresholds based on actual behavior
   - Document final values for your specific setup

---

## Conclusion

This implementation provides a robust rear-lidar-only docking strategy that:
- Uses reflective tape detection at medium range for accurate pose estimation
- Switches to direct range measurement at close range when reflectors become unreliable
- Debounces the final docking decision to avoid false positives
- Preserves the existing codebase structure with minimal changes
- Provides comprehensive debug outputs for tuning and troubleshooting

All old code has been preserved as comments for easy rollback if needed.
