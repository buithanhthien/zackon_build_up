import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

SCAN_TOPIC          = '/scan_front_filter'
I_PEAK              = 43.0
I_VALLEY            = 29.0
VALLEY_SEARCH_RANGE = 19
MAX_DETECT_RANGE    = 3.0
RUBBER_WIDTH        = 0.034
REFLECTOR_WIDTH     = 0.05
TAPE_DISTANCE       = 0.375
LRF_FORWARD_OFFSET  = 0.30
MIN_ANGLE_DEG       = -60.0
MAX_ANGLE_DEG       =  60.0


def compute_inception_angle(Li, Lj, theta):
    denom = Lj * math.cos(theta) - Li
    print(f"    inception_angle: Li={Li:.4f} Lj={Lj:.4f} theta={math.degrees(theta):.4f}deg  denom={denom:.6f}")
    if abs(denom) < 1e-6:
        print(f"    inception_angle: denom~0 -> beta=0.0")
        return 0.0
    beta = math.atan(Lj * math.sin(theta) / denom)
    print(f"    inception_angle: beta={math.degrees(beta):.4f}deg ({beta:.6f}rad)")
    return beta


def detect_reflectors(scan):
    N = len(scan.ranges)
    if len(scan.intensities) != N:
        print("  [ERROR] intensities size != ranges size")
        return []

    theta = scan.angle_increment
    max_range_geom = min(RUBBER_WIDTH, REFLECTOR_WIDTH) / (2.0 * math.sin(theta / 2.0))
    max_range = min(max_range_geom, MAX_DETECT_RANGE)
    min_angle = math.radians(MIN_ANGLE_DEG)
    max_angle = math.radians(MAX_ANGLE_DEG)
    margin = VALLEY_SEARCH_RANGE + 2

    print(f"  max_range_geom = min({RUBBER_WIDTH},{REFLECTOR_WIDTH}) / (2*sin({math.degrees(theta/2):.4f}deg)) = {max_range_geom:.4f}")
    print(f"  max_range = min({max_range_geom:.4f}, {MAX_DETECT_RANGE}) = {max_range:.4f}")
    print(f"  scan window: [{MIN_ANGLE_DEG}, {MAX_ANGLE_DEG}] deg  margin={margin}  N={N}")

    reflectors = []
    i = margin
    while i < N - margin:
        angle = scan.angle_min + i * scan.angle_increment
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi

        if angle < min_angle or angle > max_angle:
            i += 1
            continue

        I_i = scan.intensities[i]
        if I_i < I_PEAK:
            i += 1
            continue

        print(f"\n  [CANDIDATE] idx={i}  angle={math.degrees(angle):.2f}deg  I={I_i:.1f} >= i_peak={I_PEAK}")

        is_max = True
        for k in range(i - 1, max(0, i - VALLEY_SEARCH_RANGE) - 1, -1):
            if scan.intensities[k] > I_i:
                print(f"    local_max FAIL left: idx={k} I={scan.intensities[k]:.1f} > {I_i:.1f}")
                is_max = False
                break
        if is_max:
            for k in range(i + 1, min(N - 1, i + VALLEY_SEARCH_RANGE) + 1):
                if scan.intensities[k] > I_i:
                    print(f"    local_max FAIL right: idx={k} I={scan.intensities[k]:.1f} > {I_i:.1f}")
                    is_max = False
                    break
        if not is_max:
            i += 1
            continue
        print(f"    local_max PASS")

        vl_idx, vl_val = i - 1, scan.intensities[i - 1]
        for k in range(i - 2, max(0, i - VALLEY_SEARCH_RANGE) - 1, -1):
            if scan.intensities[k] < vl_val:
                vl_val, vl_idx = scan.intensities[k], k

        vr_idx, vr_val = i + 1, scan.intensities[i + 1]
        for k in range(i + 2, min(N - 1, i + VALLEY_SEARCH_RANGE) + 1):
            if scan.intensities[k] < vr_val:
                vr_val, vr_idx = scan.intensities[k], k

        print(f"    valley_left:  idx={vl_idx} I={vl_val:.1f}  valley_right: idx={vr_idx} I={vr_val:.1f}  threshold={I_VALLEY}")
        if vl_val > I_VALLEY or vr_val > I_VALLEY:
            print(f"    valley FAIL")
            i += 1
            continue
        print(f"    valley PASS")

        r_i = scan.ranges[i]
        valid_ri = math.isfinite(r_i) and scan.range_min <= r_i <= scan.range_max
        print(f"    r_i={r_i:.4f}  valid={valid_ri}  max_range={max_range:.4f}")
        if not valid_ri or r_i > max_range:
            print(f"    range r_i FAIL")
            i += 1
            continue

        r_j = scan.ranges[i - 1]
        valid_rj = math.isfinite(r_j) and scan.range_min <= r_j <= scan.range_max
        print(f"    r_j(i-1)={r_j:.4f}  valid={valid_rj}")
        if not valid_rj:
            print(f"    range r_j FAIL")
            i += 1
            continue

        beta = compute_inception_angle(r_i, r_j, theta)
        if abs(beta) < 0.05:
            print(f"    inception_angle FAIL: |beta|={abs(beta):.4f} < 0.05")
            i += 1
            continue
        print(f"    inception_angle PASS")

        rplidar_angle = scan.angle_min + i * scan.angle_increment
        ros_angle = math.pi - rplidar_angle
        while ros_angle >  math.pi: ros_angle -= 2 * math.pi
        while ros_angle < -math.pi: ros_angle += 2 * math.pi

        x = r_i * math.cos(ros_angle)
        y = r_i * math.sin(ros_angle)
        print(f"    rplidar_angle={math.degrees(rplidar_angle):.2f}deg  ros_angle={math.degrees(ros_angle):.2f}deg")
        print(f"    x = {r_i:.4f} * cos({math.degrees(ros_angle):.2f}deg) = {x:.4f}")
        print(f"    y = {r_i:.4f} * sin({math.degrees(ros_angle):.2f}deg) = {y:.4f}")
        print(f"    I_valley_min = min({vl_val:.1f}, {vr_val:.1f}) = {min(vl_val,vr_val):.1f}")
        print(f"    => REFLECTOR ACCEPTED")

        reflectors.append({
            'peak_idx': i, 'valley_l_idx': vl_idx, 'valley_r_idx': vr_idx,
            'I_peak': I_i, 'I_valley': min(vl_val, vr_val),
            'L_peak': r_i, 'beta': beta, 'x': x, 'y': y,
        })
        i = vr_idx + 1

    return reflectors


def compute_dock_pose(left, right):
    measured_dist = math.hypot(left['x'] - right['x'], left['y'] - right['y'])
    tolerance = TAPE_DISTANCE * 0.2
    print(f"  measured_dist = hypot({left['x']:.4f}-{right['x']:.4f}, {left['y']:.4f}-{right['y']:.4f}) = {measured_dist:.4f}")
    print(f"  expected={TAPE_DISTANCE}  tolerance={tolerance:.4f}  |diff|={abs(measured_dist - TAPE_DISTANCE):.4f}")
    if abs(measured_dist - TAPE_DISTANCE) > tolerance:
        print(f"  dock_pose FAIL: distance mismatch")
        return

    theta_L = math.atan2(left['y'],  left['x'])
    theta_R = math.atan2(right['y'], right['x'])
    print(f"  theta_L = atan2({left['y']:.4f}, {left['x']:.4f}) = {math.degrees(theta_L):.4f}deg")
    print(f"  theta_R = atan2({right['y']:.4f}, {right['x']:.4f}) = {math.degrees(theta_R):.4f}deg")

    phi_L = math.pi / 2.0 - left['beta']  - theta_L
    phi_R = math.pi / 2.0 - right['beta'] - theta_R
    phi_m = (phi_L + phi_R) / 2.0
    print(f"  phi_L = pi/2 - beta_L({math.degrees(left['beta']):.4f}deg) - theta_L({math.degrees(theta_L):.4f}deg) = {math.degrees(phi_L):.4f}deg")
    print(f"  phi_R = pi/2 - beta_R({math.degrees(right['beta']):.4f}deg) - theta_R({math.degrees(theta_R):.4f}deg) = {math.degrees(phi_R):.4f}deg")
    print(f"  phi_m = (phi_L + phi_R) / 2 = {math.degrees(phi_m):.4f}deg")

    mid_x = (left['x'] + right['x']) / 2.0 + LRF_FORWARD_OFFSET
    mid_y = (left['y'] + right['y']) / 2.0
    qz = math.sin(phi_m / 2.0)
    qw = math.cos(phi_m / 2.0)
    print(f"  position.x = ({left['x']:.4f}+{right['x']:.4f})/2 + {LRF_FORWARD_OFFSET} = {mid_x:.4f}")
    print(f"  position.y = ({left['y']:.4f}+{right['y']:.4f})/2 = {mid_y:.4f}")
    print(f"  orientation: qz=sin({math.degrees(phi_m/2):.4f}deg)={qz:.4f}  qw=cos(...)={qw:.4f}")
    print(f"  => DOCK POSE ACCEPTED")


class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_lidar_internal_computation')
        self.sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.cb, 10)
        print(f"Listening on {SCAN_TOPIC} ...")

    def cb(self, msg):
        print("\n" + "=" * 70)
        print(f"SCAN  angle_min={math.degrees(msg.angle_min):.2f}deg  angle_max={math.degrees(msg.angle_max):.2f}deg  increment={math.degrees(msg.angle_increment):.4f}deg  beams={len(msg.ranges)}")
        print("--- detectReflectors ---")
        reflectors = detect_reflectors(msg)
        print(f"\n  => {len(reflectors)} reflector(s) found")

        if len(reflectors) == 2:
            right, left = sorted(reflectors, key=lambda r: r['peak_idx'])
            print(f"\n--- computeDockPose ---")
            print(f"  right: idx={right['peak_idx']} x={right['x']:.4f} y={right['y']:.4f} beta={math.degrees(right['beta']):.4f}deg")
            print(f"  left:  idx={left['peak_idx']}  x={left['x']:.4f} y={left['y']:.4f} beta={math.degrees(left['beta']):.4f}deg")
            compute_dock_pose(left, right)
        elif len(reflectors) != 0:
            print(f"  => {len(reflectors)} reflectors, need exactly 2 — skipping dock pose")


def main():
    rclpy.init()
    node = DebugNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
