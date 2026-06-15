import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped   # ← changed from Twist
import math
import time

class AutoNav(Node):
    def __init__(self):
        super().__init__('auto_nav')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # ← TwistStamped now
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.state       = 'forward'
        self.state_start = time.time()
        self.last_scan   = None

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('AutoNav started')

    def scan_callback(self, msg):
        self.last_scan = msg

    def control_loop(self):
        if self.last_scan is None:
            self.get_logger().info('Waiting for /scan...', throttle_duration_sec=2.0)
            return

        ranges = self.last_scan.ranges
        n      = len(ranges)

        def safe(v):
            return v if math.isfinite(v) and v > 0.01 else 10.0

        front       = min(safe(ranges[i % n]) for i in range(-20,  20))
        front_left  = min(safe(ranges[i % n]) for i in range( 20,  60))
        front_right = min(safe(ranges[i % n]) for i in range(-60, -20))
        left        = min(safe(ranges[i % n]) for i in range( 60, 120))
        right       = min(safe(ranges[i % n]) for i in range(-120,-60))

        # Log distances so you can see what lidar actually reads
        self.get_logger().info(
            f'front={front:.2f} fl={front_left:.2f} fr={front_right:.2f} '
            f'l={left:.2f} r={right:.2f} state={self.state}',
            throttle_duration_sec=0.5)

        DANGER = 0.30
        WARN   = 0.55
        CLEAR  = 0.70
        elapsed = time.time() - self.state_start

        if self.state == 'recover':
            if elapsed < 1.2:
                self.publish(-0.15, 0.0)
            else:
                direction = 1.0 if left > right else -1.0
                self.switch_state('arc_left' if direction > 0 else 'arc_right')
            return

        if self.state == 'forward':
            if front < DANGER or front_left < DANGER or front_right < DANGER:
                self.switch_state('recover')
            elif front < WARN:
                if front_left < front_right:
                    self.switch_state('arc_right')
                else:
                    self.switch_state('arc_left')
            elif front_left < WARN:
                self.switch_state('arc_right')
            elif front_right < WARN:
                self.switch_state('arc_left')
            else:
                correction = 0.08 * (right - left)
                self.publish(0.22, correction)

        elif self.state in ('arc_left', 'arc_right'):
            direction = 1.0 if self.state == 'arc_left' else -1.0
            if elapsed > 0.8 and front > CLEAR and front_left > WARN and front_right > WARN:
                self.switch_state('forward')
            elif front < DANGER:
                self.switch_state('recover')
            else:
                self.publish(0.13, direction * 0.6)

    def switch_state(self, new_state):
        self.get_logger().info(f'{self.state} → {new_state}')
        self.state       = new_state
        self.state_start = time.time()

    def publish(self, linear, angular):
        msg = TwistStamped()                        # ← TwistStamped
        msg.header.stamp = self.get_clock().now().to_msg()  # ← needs a timestamp
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = linear               # ← now msg.twist.linear not msg.linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
