#!/usr/bin/env python3
"""
Smooth, car-like autonomous explorer for a TurtleBot3 (ROS 2 Jazzy).

Why this is different from a bang-bang state machine:
    The old version snapped between discrete states (forward / arc / recover)
    with fixed turn rates, so every correction was a hard lurch -> the robot
    "went here and there" and spun around at obstacles.

    This version uses CONTINUOUS PROPORTIONAL CONTROL:
      - speed scales with how clear the path ahead is  (slow down near things)
      - steering is a smooth function of left/right clearance
            * stay centered in a corridor  (the "stay in lane" feel)
            * gently curve toward open space when something is ahead
      - it only falls back to a reverse-and-turn recovery at a TRUE dead end

    Result: straight-line cruising with gentle, car-like corrections.

How to use it for mapping:
    Run this WHILE slam_toolbox is mapping to build a map hands-free (no
    teleop), then save the map and hand navigation over to Nav2. Launch it
    with sim time so its timestamps line up with Gazebo and SLAM:

        python3 auto_nav.py --ros-args -p use_sim_time:=true
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class AutoNav(Node):
    def __init__(self):
        super().__init__('auto_nav')

        # ---- tunables (start here if behaviour needs adjusting) ----
        self.MAX_V    = 0.22    # m/s   TurtleBot3 burger top speed
        self.MIN_V    = 0.04    # m/s   crawl so steering keeps authority in tight spots
        self.MAX_W    = 1.4     # rad/s steering cap (below burger max -> smoother)

        self.DANGER   = 0.35    # m   closer than this straight ahead -> recovery
        self.WARN     = 0.70    # m   start curving away below this clearance
        self.CRUISE   = 1.20    # m   this much clearance ahead -> full speed

        self.K_CENTER = 0.6     # corridor-centering gain
        self.K_AVOID  = 2.2     # how hard to steer away when something is ahead
        self.SMOOTH   = 0.35    # command low-pass (0 = frozen, 1 = instant/jerky)

        # ---- state ----
        self.scan = None
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.mode = 'drive'             # 'drive' or 'recover'
        self.mode_start = self._now()

        self.sub = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)   # 20 Hz
        self.get_logger().info('AutoNav (smooth car-like explorer) started')

    # ---------- helpers ----------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, msg):
        self.scan = msg

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def sector_min(self, lo_deg, hi_deg):
        """Smallest valid range (m) within [lo_deg, hi_deg].
        0 deg = straight ahead, +deg = LEFT (CCW), per REP-103.
        Computed from angle_min/angle_increment so it works at any scan
        resolution (not hard-coded to a 360-point lidar)."""
        s = self.scan
        best = s.range_max
        lo, hi = math.radians(lo_deg), math.radians(hi_deg)
        for i, r in enumerate(s.ranges):
            if not (math.isfinite(r) and r > s.range_min):
                continue
            ang = s.angle_min + i * s.angle_increment
            ang = math.atan2(math.sin(ang), math.cos(ang))   # wrap to [-pi, pi]
            if lo <= ang <= hi and r < best:
                best = r
        return best

    # ---------- main control ----------
    def control_loop(self):
        if self.scan is None:
            self.get_logger().info('waiting for /scan...', throttle_duration_sec=2.0)
            return

        front  = self.sector_min(-25,  25)
        fleft  = self.sector_min( 15,  55)
        fright = self.sector_min(-55, -15)
        left   = self.sector_min( 40,  90)
        right  = self.sector_min(-90, -40)

        self.get_logger().info(
            f'front={front:.2f} fl={fleft:.2f} fr={fright:.2f} '
            f'l={left:.2f} r={right:.2f} mode={self.mode}',
            throttle_duration_sec=0.5)

        elapsed = self._now() - self.mode_start

        # ----- recovery: only at a genuine dead end -----
        if self.mode == 'recover':
            if elapsed < 0.8:
                self.send(-0.10, 0.0)                          # back straight up
            else:
                turn_dir = 1.0 if left > right else -1.0       # pivot toward open side
                self.send(0.0, turn_dir * self.MAX_W)
                if front > self.WARN and elapsed > 1.4:
                    self.set_mode('drive')
            return

        if front < self.DANGER:
            self.set_mode('recover')
            self.send(0.0, 0.0)
            return

        # ----- normal driving: continuous proportional control -----
        # speed: full when clearance ahead is large, crawl as it shrinks
        speed = self.MAX_V * self._clamp(
            (front - self.DANGER) / (self.CRUISE - self.DANGER), 0.0, 1.0)
        speed = max(speed, self.MIN_V)

        # steering: positive => more room on left => turn left (CCW).
        # This both stays centered in a corridor AND eases away from the nearer wall.
        turn = self.K_CENTER * (left - right)

        # if something is ahead, curve toward the clearer front side and ease off speed
        if front < self.WARN:
            urgency = (self.WARN - front) / (self.WARN - self.DANGER)
            turn += self.K_AVOID * urgency * (1.0 if fleft > fright else -1.0)
            speed *= 0.55

        self.send(self._clamp(speed, 0.0, self.MAX_V),
                  self._clamp(turn, -self.MAX_W, self.MAX_W))

    def set_mode(self, m):
        if m != self.mode:
            self.get_logger().info(f'{self.mode} -> {m}')
        self.mode = m
        self.mode_start = self._now()

    def send(self, v, w):
        # low-pass the commands so motion flows instead of lurching
        self.v_cmd += self.SMOOTH * (v - self.v_cmd)
        self.w_cmd += self.SMOOTH * (w - self.w_cmd)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = self.v_cmd
        msg.twist.angular.z = self.w_cmd
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = AutoNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
