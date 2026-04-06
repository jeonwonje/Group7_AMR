# testing for dynamic station
# tag 0 placed beside hole, tag 2 placed inside tin
# uses tag 0 to dock, waits for tag 2 to come into view before logging "Launch"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from apriltag_msgs.msg import AprilTagDetectionArray
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import RPi.GPIO as GPIO
import time
import threading

EXPLORE = 'explore'
DOCK = 'dock'
DOCKED = 'docked'
ESCAPE = 'escape'

DOCK_TAG_ID = 0
LAUNCH_TAG_ID = 2
DOCK_DISTANCE = 0.30
DOCK_TOL = 0.02
IMAGE_W = 800

SERVO_PIN = 12
DURATION = 0.909


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, scan_qos)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detections_callback, 10)

        self._scan = None
        self._stuck_counter = 0
        self._state = EXPLORE

        self._escape_phase = 'backup'
        self._escape_timer = 0
        self._escape_backup_ticks = 15
        self._escape_rotate_ticks = 20
        self._escape_direction = 1.0
        self._prev_state = EXPLORE

        self._dock_tag_centre_x = None
        self._dock_tag_size_px = None
        self._launch_triggered = False

        # servo setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self._pwm = GPIO.PWM(SERVO_PIN, 50)
        self._pwm.start(0)

        self.create_timer(0.1, self.run)

    def scan_callback(self, msg):
        self._scan = list(msg.ranges)

    def detections_callback(self, msg):
        self._dock_tag_centre_x = None
        self._dock_tag_size_px = None

        for det in msg.detections:
            if det.id == DOCK_TAG_ID:
                self._dock_tag_centre_x = det.centre.x
                xs = [c.x for c in det.corners]
                ys = [c.y for c in det.corners]
                w = max(xs) - min(xs)
                h = max(ys) - min(ys)
                self._dock_tag_size_px = (w + h) / 2.0

            if det.id == LAUNCH_TAG_ID and self._state == DOCKED:
                if not self._launch_triggered:
                    self._launch_triggered = True
                    threading.Thread(target=self.fire_servo, daemon=True).start()

    def fire_servo(self):
        self.get_logger().info(f'firing servo')
        self._pwm.ChangeDutyCycle(5.0)
        time.sleep(DURATION)
        self._pwm.ChangeDutyCycle(0)
        self.get_logger().info('servo done')

    def estimate_distance_from_tag(self):
        if self._dock_tag_size_px is None or self._dock_tag_size_px < 1:
            return None
        PHYSICAL_TAG_M = 0.173
        FOCAL_PX = 530.0
        return (PHYSICAL_TAG_M * FOCAL_PX) / self._dock_tag_size_px

    def run(self):
        if self._state == EXPLORE:
            if self._dock_tag_centre_x is not None:
                self.get_logger().info('tag 0 detected -> switching to DOCK')
                self._state = DOCK
                return
            self.explore()

        elif self._state == DOCK:
            self.dock()

        elif self._state == DOCKED:
            self.docked()

        elif self._state == ESCAPE:
            self.escape()

    def explore(self):
        if self._scan is None:
            return

        ranges = [r if 0.1 < r < 10.0 else 0.0 for r in self._scan]
        front = ranges[0:30] + ranges[-30:]
        front_valid = [r for r in front if r > 0.1]
        cmd = Twist()

        if self._stuck_counter > 20:
            left = [r for r in ranges[0:30] if r > 0.1]
            right = [r for r in ranges[-30:] if r > 0.1]
            left_avg = np.mean(left) if left else 0.0
            right_avg = np.mean(right) if right else 0.0

            self._escape_direction = 1.0 if left_avg > right_avg else -1.0
            self._escape_phase = 'backup'
            self._escape_timer = 0
            self._stuck_counter = 0
            self._prev_state = self._state
            self._state = ESCAPE
            self.get_logger().info('stucked, entering escape')
            return

        elif not front_valid or min(front_valid) > 0.25:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
            self._stuck_counter = 0
        else:
            cmd.linear.x = 0.0
            left = [r for r in ranges[0:30] if r > 0.1]
            right = [r for r in ranges[-30:] if r > 0.1]
            left_min = min(left) if left else 10.0
            right_min = min(right) if right else 10.0
            cmd.angular.z = -0.5 if left_min < right_min else 0.5
            self._stuck_counter += 1

        self._cmd_pub.publish(cmd)
        self.get_logger().info('exploring...')

    def dock(self):
        cmd = Twist()

        if self._dock_tag_centre_x is None:
            cmd.angular.z = 0.1
            self._cmd_pub.publish(cmd)
            self.get_logger().info('tag 0 lost, searching...')
            return

        error_x = self._dock_tag_centre_x - (IMAGE_W / 2.0)
        cmd.angular.z = float(np.clip(-0.003 * error_x, -0.5, 0.5))

        dist = self.estimate_distance_from_tag()
        if dist is None:
            self._cmd_pub.publish(cmd)
            return

        self.get_logger().info(f'docking -> dist: {dist:.3f} m, target: {DOCK_DISTANCE} m')

        dist_error = dist - DOCK_DISTANCE
        if abs(dist_error) < DOCK_TOL:
            self.get_logger().info('docked, waiting for tag 2')
            self._state = DOCKED
            self.stop()
            return

        cmd.linear.x = float(np.clip(0.4 * dist_error, -0.15, 0.15))
        self._cmd_pub.publish(cmd)

    def docked(self):
        self.stop()
        if not self._launch_triggered:
            self.get_logger().info('docked, waiting for tag 2', throttle_duration_sec=2.0)

    def escape(self):
        cmd = Twist()

        if self._escape_phase == 'backup':
            cmd.linear.x = -0.1
            cmd.angular.z = 0.0
            self._escape_timer += 1
            if self._escape_timer >= self._escape_backup_ticks:
                self._escape_phase = 'rotate'
                self._escape_timer = 0
                self.get_logger().info('backup done, rotating')

        elif self._escape_phase == 'rotate':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 * self._escape_direction
            self._escape_timer += 1
            if self._escape_timer >= self._escape_rotate_ticks:
                self._state = self._prev_state
                self._escape_timer = 0
                self.get_logger().info('escape done, resuming')

        self._cmd_pub.publish(cmd)

    def stop(self):
        self._cmd_pub.publish(Twist())

    def destroy_node(self):
        self._pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()