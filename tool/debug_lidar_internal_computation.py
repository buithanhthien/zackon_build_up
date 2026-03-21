import math
import os
import sys
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH

_PARAMS_FILE = os.path.join(SOURCE_PATH, 'src/lidar_dock_detector/config/docking_params.yaml')
with open(_PARAMS_FILE) as _f:
    _rp = yaml.safe_load(_f)['docking_server']['ros__parameters']['reflective_tape_dock']

SCAN_TOPIC          = _rp['scan_topic']
I_PEAK              = float(_rp['i_peak'])              # Min intensity to classify a beam as hitting reflective tape
I_VALLEY            = float(_rp['i_valley'])            # Max intensity allowed in beams flanking a peak (must be < I_PEAK)
VALLEY_SEARCH_RANGE = int(_rp['valley_search_range'])   # Number of beams to search on each side of a peak candidate
MAX_DETECT_RANGE    = float(_rp['max_detect_range'])    # Max range (m) at which the dock can be detected
RUBBER_WIDTH        = float(_rp['rubber_width'])        # Width (m) of the non-reflective gap between tape strips
REFLECTOR_WIDTH     = float(_rp['reflector_width'])     # Width (m) of each reflective tape strip
TAPE_DISTANCE       = float(_rp['tape_distance'])       # Expected center-to-center distance (m) between the two tape strips
LRF_FORWARD_OFFSET  = float(_rp['lrf_forward_offset'])  # Distance (m) from LiDAR to robot center along forward axis
MIN_ANGLE_DEG       = float(_rp['min_detection_angle_deg'])  # Left boundary (deg) of the angular search window
MAX_ANGLE_DEG       = float(_rp['max_detection_angle_deg'])  # Right boundary (deg) of the angular search window


def _inception_angle(Li, Lj, theta):
    # Eq.(3a): computes beta, the angle of incidence of the LiDAR beam on the reflective tape surface.
    # Li    = range (m) of beam i (peak beam hitting tape center)
    # Lj    = range (m) of beam j=i-1 (adjacent beam just before the peak)
    # theta = angular step between consecutive beams (scan.angle_increment, rad)
    # denom near-zero means beam is nearly parallel to tape surface — no reliable angle
    denom = Lj * math.cos(theta) - Li
    print(f"    [inception_angle] Li={Li:.4f} Lj={Lj:.4f} theta={math.degrees(theta):.4f}deg  denom={denom:.6f}")
    if abs(denom) < 1e-6:
        print(f"    [inception_angle] denom~0 => beta=0.0  SKIP")
        return 0.0
    beta = math.atan(Lj * math.sin(theta) / denom)  # incidence angle of beam on tape surface (rad)
    print(f"    [inception_angle] beta={math.degrees(beta):.4f}deg ({beta:.6f}rad)")
    return beta


def _detect_reflectors(scan):
    N = len(scan.ranges)
    if len(scan.intensities) != N:
        print("  [ERROR] intensities size != ranges size")
        return []

    theta = scan.angle_increment                                                          # angular step between beams (rad)
    max_range_geom = min(RUBBER_WIDTH, REFLECTOR_WIDTH) / (2.0 * math.sin(theta / 2.0)) # Eq.(8a): geometric max detectable range based on tape/gap width and beam spacing
    max_range = min(max_range_geom, MAX_DETECT_RANGE)                                    # effective max range: tighter of geometric limit and configured limit
    min_angle = math.radians(MIN_ANGLE_DEG)                                              # left boundary of search window (rad)
    max_angle = math.radians(MAX_ANGLE_DEG)                                              # right boundary of search window (rad)
    margin = VALLEY_SEARCH_RANGE + 2                                                     # index margin to avoid out-of-bounds when searching valleys

    print(f"  [detectReflectors] N={N}  theta={math.degrees(theta):.4f}deg  margin={margin}")
    print(f"  [detectReflectors] max_range_geom={max_range_geom:.4f}  max_range={max_range:.4f}")
    print(f"  [detectReflectors] angle_window=[{MIN_ANGLE_DEG}, {MAX_ANGLE_DEG}]deg")

    reflectors = []
    i = margin
    while i < N - margin:
        angle = scan.angle_min + i * scan.angle_increment  # bearing of beam i in scan frame (rad), normalized to [-pi, pi]
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi

        if angle < min_angle or angle > max_angle:
            i += 1
            continue

        I_i = scan.intensities[i]  # intensity of beam i; high value indicates reflective tape
        if I_i < I_PEAK:
            i += 1
            continue

        print(f"\n  [CANDIDATE] idx={i}  angle={math.degrees(angle):.2f}deg  I={I_i:.1f} >= i_peak={I_PEAK}")

        is_max = True  # True if beam i is the local intensity maximum within valley_search_range
        for k in range(i - 1, max(0, i - VALLEY_SEARCH_RANGE) - 1, -1):
            if scan.intensities[k] > I_i:
                print(f"    [local_max] FAIL left  idx={k} I={scan.intensities[k]:.1f} > {I_i:.1f}")
                is_max = False
                break
        if is_max:
            for k in range(i + 1, min(N - 1, i + VALLEY_SEARCH_RANGE) + 1):
                if scan.intensities[k] > I_i:
                    print(f"    [local_max] FAIL right idx={k} I={scan.intensities[k]:.1f} > {I_i:.1f}")
                    is_max = False
                    break
        if not is_max:
            i += 1
            continue
        print(f"    [local_max] PASS")

        vl_idx, vl_val = i - 1, scan.intensities[i - 1]  # index and intensity of the left valley minimum
        for k in range(i - 2, max(0, i - VALLEY_SEARCH_RANGE) - 1, -1):
            if scan.intensities[k] < vl_val:
                vl_val, vl_idx = scan.intensities[k], k

        vr_idx, vr_val = i + 1, scan.intensities[i + 1]  # index and intensity of the right valley minimum
        for k in range(i + 2, min(N - 1, i + VALLEY_SEARCH_RANGE) + 1):
            if scan.intensities[k] < vr_val:
                vr_val, vr_idx = scan.intensities[k], k

        print(f"    [valley] left: idx={vl_idx} I={vl_val:.1f}  right: idx={vr_idx} I={vr_val:.1f}  threshold={I_VALLEY}")
        if vl_val > I_VALLEY or vr_val > I_VALLEY:
            print(f"    [valley] FAIL")
            i += 1
            continue
        print(f"    [valley] PASS")

        r_i = scan.ranges[i]                                                            # range (m) of the peak beam
        valid_ri = math.isfinite(r_i) and scan.range_min <= r_i <= scan.range_max
        print(f"    [range] r_i={r_i:.4f}  valid={valid_ri}  max_range={max_range:.4f}")
        if not valid_ri or r_i > max_range:
            print(f"    [range] r_i FAIL")
            i += 1
            continue

        r_j = scan.ranges[i - 1]                                                        # range (m) of adjacent beam (i-1), used in inception angle formula
        valid_rj = math.isfinite(r_j) and scan.range_min <= r_j <= scan.range_max
        print(f"    [range] r_j(i-1)={r_j:.4f}  valid={valid_rj}")
        if not valid_rj:
            print(f"    [range] r_j FAIL")
            i += 1
            continue
        print(f"    [range] PASS")

        beta = _inception_angle(r_i, r_j, theta)  # incidence angle of beam on tape surface (rad), Eq.(3a)
        if abs(beta) < 0.05:
            print(f"    [inception_angle] FAIL |beta|={abs(beta):.4f} < 0.05")
            i += 1
            continue
        print(f"    [inception_angle] PASS")

        rplidar_angle = scan.angle_min + i * scan.angle_increment  # raw beam angle in RPLiDAR frame (CW positive)
        ros_angle = math.pi - rplidar_angle                        # converted to ROS frame (CCW positive)
        while ros_angle >  math.pi: ros_angle -= 2 * math.pi
        while ros_angle < -math.pi: ros_angle += 2 * math.pi

        x = r_i * math.cos(ros_angle)  # reflector x position in laser frame (m), forward = positive
        y = r_i * math.sin(ros_angle)  # reflector y position in laser frame (m), left = positive
        print(f"    [position] rplidar_angle={math.degrees(rplidar_angle):.2f}deg  ros_angle={math.degrees(ros_angle):.2f}deg")
        print(f"    [position] x={r_i:.4f}*cos({math.degrees(ros_angle):.2f}deg)={x:.4f}  y={r_i:.4f}*sin(...)={y:.4f}")
        print(f"    [reflector] I_peak={I_i:.1f}  I_valley={min(vl_val,vr_val):.1f}  L_peak={r_i:.4f}  beta={math.degrees(beta):.4f}deg  => ACCEPTED")

        reflectors.append({
            'peak_idx': i, 'valley_l_idx': vl_idx, 'valley_r_idx': vr_idx,
            'I_peak': I_i, 'I_valley': min(vl_val, vr_val),
            'L_peak': r_i, 'beta': beta, 'x': x, 'y': y,
        })
        i = vr_idx + 1

    return reflectors


def _compute_dock_pose(left, right):
    measured_dist = math.hypot(left['x'] - right['x'], left['y'] - right['y'])  # Euclidean distance between the two detected reflectors (m)
    tolerance = TAPE_DISTANCE * 0.2                                               # allowed ±20% deviation from expected tape_distance
    print(f"  [computeDockPose] measured_dist={measured_dist:.4f}  expected={TAPE_DISTANCE}  tolerance={tolerance:.4f}  |diff|={abs(measured_dist-TAPE_DISTANCE):.4f}")
    if abs(measured_dist - TAPE_DISTANCE) > tolerance:
        print(f"  [computeDockPose] FAIL distance mismatch")
        return

    theta_L = math.atan2(left['y'],  left['x'])   # bearing angle to left reflector from laser origin (rad)
    theta_R = math.atan2(right['y'], right['x'])  # bearing angle to right reflector from laser origin (rad)
    print(f"  [computeDockPose] theta_L=atan2({left['y']:.4f},{left['x']:.4f})={math.degrees(theta_L):.4f}deg")
    print(f"  [computeDockPose] theta_R=atan2({right['y']:.4f},{right['x']:.4f})={math.degrees(theta_R):.4f}deg")

    phi_L = math.pi / 2.0 - left['beta']  - theta_L   # dock surface normal angle estimated from left tape (rad), Eq.(9)
    phi_R = math.pi / 2.0 - right['beta'] - theta_R   # dock surface normal angle estimated from right tape (rad), Eq.(9)
    phi_m = (phi_L + phi_R) / 2.0                     # averaged dock heading angle (rad)
    print(f"  [computeDockPose] phi_L=pi/2 - {math.degrees(left['beta']):.4f}deg - {math.degrees(theta_L):.4f}deg = {math.degrees(phi_L):.4f}deg")
    print(f"  [computeDockPose] phi_R=pi/2 - {math.degrees(right['beta']):.4f}deg - {math.degrees(theta_R):.4f}deg = {math.degrees(phi_R):.4f}deg")
    print(f"  [computeDockPose] phi_m=(phi_L+phi_R)/2 = {math.degrees(phi_m):.4f}deg")

    pos_x = (left['x'] + right['x']) / 2.0 + LRF_FORWARD_OFFSET  # dock x in laser frame: midpoint of tapes + LiDAR-to-robot offset (m)
    pos_y = (left['y'] + right['y']) / 2.0                        # dock y in laser frame: midpoint of tapes (m)
    qz = math.sin(phi_m / 2.0)  # quaternion z component encoding dock heading
    qw = math.cos(phi_m / 2.0)  # quaternion w component encoding dock heading
    print(f"  [computeDockPose] position.x=({left['x']:.4f}+{right['x']:.4f})/2+{LRF_FORWARD_OFFSET}={pos_x:.4f}")
    print(f"  [computeDockPose] position.y=({left['y']:.4f}+{right['y']:.4f})/2={pos_y:.4f}")
    print(f"  [computeDockPose] orientation qz=sin({math.degrees(phi_m/2):.4f}deg)={qz:.4f}  qw={qw:.4f}")
    print(f"  [computeDockPose] => DOCK POSE ACCEPTED")


class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_lidar_internal_computation')
        self.sub = self.create_subscription(LaserScan, SCAN_TOPIC, self._cb, 10)
        print(f"Params loaded from: {_PARAMS_FILE}")
        print(f"Listening on {SCAN_TOPIC} ...")

    def _cb(self, msg):
        print("\033[2J\033[H", end="")  # clear terminal, move cursor to top
        print("=" * 70)
        print(f"[SCAN] angle_min={math.degrees(msg.angle_min):.2f}deg  angle_max={math.degrees(msg.angle_max):.2f}deg  increment={math.degrees(msg.angle_increment):.4f}deg  beams={len(msg.ranges)}")

        print("\n--- detectReflectors ---")
        reflectors = _detect_reflectors(msg)
        print(f"\n  => {len(reflectors)} reflector(s) found")

        if len(reflectors) == 2:
            right, left = sorted(reflectors, key=lambda r: r['peak_idx'])
            print(f"\n--- computeDockPose ---")
            print(f"  right: idx={right['peak_idx']}  x={right['x']:.4f}  y={right['y']:.4f}  beta={math.degrees(right['beta']):.4f}deg  I_peak={right['I_peak']:.1f}")
            print(f"  left:  idx={left['peak_idx']}   x={left['x']:.4f}  y={left['y']:.4f}  beta={math.degrees(left['beta']):.4f}deg  I_peak={left['I_peak']:.1f}")
            print()
            _compute_dock_pose(left, right)
        elif len(reflectors) != 0:
            print(f"  => need exactly 2 reflectors, got {len(reflectors)} — skipping dock pose")


def main():
    rclpy.init()
    node = DebugNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
