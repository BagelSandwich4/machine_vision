"""
Drive a Neato along a fixed route and log the AprilTags it sees.

The robot repeats: drive forward STEP_DIST -> stop -> wait WAIT_TIME ->
scan SCAN_FRAMES camera frames -> log any new tags. The run ends after
NUM_STOPS stops, after driving MAX_DIST in total, or when the laser sees
an obstacle closer than STOP_DIST ahead. The log is written to a
time-stamped CSV in the working folder.
"""
import csv
import math
import os
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from pupil_apriltags import Detector
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
import yaml

IMAGE_TOPIC = '/camera/image_raw'
ODOM_TOPIC = '/odom'
CMD_TOPIC = '/cmd_vel'
SCAN_TOPIC = '/scan'
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration_data.yaml')
TAG_SIZE = 0.107                    # meters: side length of the black square of tags
NUM_STOPS = 5                       # how many places to stop and scan
STEP_DIST = 0.30                    # meters to drive between stops
MAX_DIST = NUM_STOPS*STEP_DIST      # maximum distance for this system
SPEED = 0.08          # m/s
WAIT_TIME = 1.0       # seconds to sit still before scanning
SCAN_FRAMES = 8       # frames to look at per stop
STOP_DIST = 0.20      # meters: stop if anything in front is closer than this
FRONT_ANGLE = 15      # degrees on each side of straight ahead to check


class Scanner(Node):
    """
    ROS node that drives a fixed route, scans for AprilTags, and logs them.

    Runs a state machine with four states: 'drive', 'scan_wait', 'scan',
    and 'done'. Each tag ID is logged only the first time it is seen.

    Attributes:
        bridge : CvBridge that converts ROS Image messages to OpenCV images.
        detector (pupil_apriltags.Detector): pupil_apriltags Detector for the
            tag25h9 family.
        K (numpy.ndarray): 3x3 camera intrinsic matrix loaded from CALIB_FILE.
        D (numpy.ndarray): Lens distortion coefficients loaded from CALIB_FILE.
        cam_params (tuple of float): Camera (fx, fy, cx, cy) used for tag pose
            estimation.
        pub (rclpy.publisher.Publisher): Publisher for velocity commands on
            CMD_TOPIC.
        x (float or None): Latest robot x position from odometry (meters), or
            None.
        y (float or None): Latest robot y position from odometry (meters), or
            None.
        front_dist (float or None): Closest valid laser reading within
            FRONT_ANGLE of straight ahead (meters), or None if there is none.
        state (str): Current state machine state.
        stop (int): Index of the current stop, starting at 0.
        start_xy (tuple of float or None): Odometry (x, y) where the current
            drive segment began.
        run_start_xy (tuple of float or None): Odometry (x, y) where the whole
            run began, set on the first odometry message.
        cap_reached (bool): True once the total distance driven reaches
            MAX_DIST.
        wait_start (float): Time (seconds) when the current scan_wait began.
        frames (int): Number of camera frames processed at the current stop.
        seen (dict): Tags seen at the current stop, mapping tag ID to the tag's
            [x, y, z] position in the camera frame (meters).
        log (dict): Logged tags for the whole run, mapping tag ID to its CSV
            row.
    """

    def __init__(self):
        """Load calibration, set up subscribers and publisher, start the loop timer."""
        super().__init__('scanner')
        self.bridge = CvBridge()
        self.detector = Detector(families='tag25h9')

        # load camera calibration from the yaml file
        with open(CALIB_FILE) as f:
            calib = yaml.safe_load(f)
        self.K = np.array(calib['camera_matrix']['data']).reshape(3, 3)   # intrinsics matrix
        self.D = np.array(calib['distortion_coefficients']['data'])       # lens distortion
        # pupil_apriltags needs -> (fx, fy, cx, cy)
        self.cam_params = (self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2])

        self.create_subscription(Image, IMAGE_TOPIC, self.on_image, 1)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_timer(0.1, self.loop)

        self.x = self.y = None      # robot position from odometry
        self.front_dist = None      # closest valid laser reading in front of the robot
        self.state = 'scan_wait'
        self.stop = 0               # which stop we are at
        self.start_xy = None        # where the current drive began
        self.run_start_xy = None    # where the whole run began (set on first odom message)
        self.cap_reached = False    # True once total distance >= MAX_DIST
        self.wait_start = time.time()
        self.frames = 0
        self.seen = {}              # tag_id -> position of tag in camera frame at this stop
        self.log = {}               # tag_id -> row

    def on_odom(self, msg):
        """
        Store the robot's latest odometry position.

        The first message received also records where the run began.

        Args:
            msg (nav_msgs.msg.Odometry): Latest odometry reading.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.run_start_xy is None:
            self.run_start_xy = (self.x, self.y)

    def on_scan(self, msg):
        """
        Update the distance to the closest obstacle in front of the robot.

        Keeps readings within FRONT_ANGLE degrees of straight ahead, drops
        invalid ones (zero, inf, nan, or outside the sensor range), and
        stores the smallest in front_dist, or None if none remain.

        Args:
            msg (sensor_msgs.msg.LaserScan): Latest laser scan.
        """
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))   # wrap to [-pi, pi]
        front = ranges[np.abs(angles) <= math.radians(FRONT_ANGLE)]
        # throw out invalid readings (0.0, inf, nan)
        front = front[np.isfinite(front) & (front > msg.range_min) & (front < msg.range_max)]
        self.front_dist = float(front.min()) if front.size else None

    def blocked(self):
        """
        Check whether an obstacle is too close in front of the robot.

        Returns:
            bool: True if the closest front reading is at or below STOP_DIST,
                False otherwise or if there is no valid reading.
        """
        return self.front_dist is not None and self.front_dist <= STOP_DIST

    def on_image(self, msg):
        """
        Detect AprilTags in a camera frame while in the scan state.

        Converts the frame to grayscale, removes lens distortion, detects
        tags with pose estimation, and records each tag's camera-frame
        position in seen. Frames received in other states are ignored.

        Args:
            msg (sensor_msgs.msg.Image): Latest camera frame.
        """
        if self.state != 'scan':
            return
        # makes the image grayscale and converts to opencv image
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)
        gray = cv2.undistort(gray, self.K, self.D)  # removes lens distortion using the calibration
        # detect tags and estimate their 3D pose relative to the camera
        for tag in self.detector.detect(gray, estimate_tag_pose=True,
                                        camera_params=self.cam_params, tag_size=TAG_SIZE):
            self.seen[tag.tag_id] = tag.pose_t.flatten()  # [x, y, z] in meters, z is forward
        self.frames += 1

    def move(self, speed=0.1):
        """
        Publish a forward velocity command.

        Args:
            speed (float, optional): Forward speed in m/s. Use 0.0 to stop.
        """
        cmd = Twist()
        cmd.linear.x = speed
        self.pub.publish(cmd)

    def loop(self):
        """
        Run one step of the main state machine (called every 0.1 s).

        Does nothing until the first odometry message arrives.

        States:
            drive: Drive forward at SPEED. Go to scan_wait after driving
                STEP_DIST, when an obstacle is ahead, or once the total
                distance reaches MAX_DIST.
            scan_wait: Stay stopped for WAIT_TIME seconds, then clear the
                per-stop data and go to scan.
            scan: After SCAN_FRAMES frames, save the stop's tags. Finish
                the run if this was the last stop, MAX_DIST was reached,
                or the path is blocked; otherwise go back to drive.
            done: Finish the run.
        """
        if self.x is None:
            return  # no odometry yet

        if self.state == 'drive':
            # euclidean distance between where this drive began vs where the robot is now
            driven = math.hypot(self.x - self.start_xy[0], self.y - self.start_xy[1])
            # distance from where the whole run began
            total = math.hypot(self.x - self.run_start_xy[0], self.y - self.run_start_xy[1])
            if self.blocked():  # obstacle ahead: stop early and scan from here
                self.get_logger().warn(f'obstacle at {self.front_dist:.2f} m, stopping early')
                self.move(0.0)
                self.state = 'scan_wait'
                self.wait_start = time.time()
            elif total >= MAX_DIST:
                # driven the maximum total distance: stop, scan here one last time, then finish
                self.get_logger().warn(
                    f'reached max distance ({total:.2f} m), scanning one last time')
                self.move(0.0)
                self.cap_reached = True
                self.state = 'scan_wait'
                self.wait_start = time.time()
            elif driven >= STEP_DIST:
                # if this dist is greater than the step, stop and wait before scanning
                self.move(0.0)
                self.state = 'scan_wait'
                self.wait_start = time.time()
            else:
                self.move(SPEED)  # if not at the step dist yet, keep driving

        elif self.state == 'scan_wait':
            # stop and wait WAIT_TIME seconds, then switch to SCAN state
            self.move(0.0)
            if time.time() - self.wait_start >= WAIT_TIME:
                self.frames = 0
                self.seen = {}
                self.state = 'scan'
                self.get_logger().info(f'stop {self.stop}: scanning for tags')

        elif self.state == 'scan':
            # wait until it gets SCAN_FRAMES frames, then save the tags and either
            # finish the run or move to the next stop
            if self.frames >= SCAN_FRAMES:
                self.save_stop()
                self.stop += 1
                if self.stop >= NUM_STOPS or self.cap_reached:
                    self.get_logger().info('finished run')
                    self.finish()
                elif self.blocked():  # can't drive further, so end the run
                    self.get_logger().warn('path blocked, ending run early')
                    self.finish()
                else:
                    self.start_xy = (self.x, self.y)
                    self.state = 'drive'
                    self.get_logger().info(f'driving to stop {self.stop}')
        elif self.state == 'done':
            self.finish()
            return

    def save_stop(self):
        """
        Add tags seen at the current stop to the run log.

        Only the first sighting of each tag ID is kept. Each row holds
        the tag ID, stop number, time, robot odometry x/y, and the tag's
        camera-frame position (cam_x, cam_y, cam_z).
        """
        for tag_id, t in self.seen.items():
            if tag_id not in self.log:  # ignore duplicates
                # adds tag to the log with its ID, stop number, time, x/y coordinates,
                # and camera-frame position
                self.log[tag_id] = {'tag_id': tag_id, 'stop': self.stop,
                                    'time': time.strftime('%H:%M:%S'),
                                    'x': round(self.x, 2), 'y': round(self.y, 2),
                                    'cam_x': round(float(t[0]), 3),
                                    'cam_y': round(float(t[1]), 3),
                                    'cam_z': round(float(t[2]), 3)}
                self.get_logger().info(f'stop {self.stop}: found tag {tag_id} at {t[2]:.2f} m')

    def finish(self):
        """
        Stop the robot, write the log to CSV, and end the program.

        The log is saved as run_YYYYMMDD_HHMMSS.csv in the current working
        directory.

        Raises:
            SystemExit: Stop rclpy.spin in main (this always occurs)
        """
        self.move(0.0)
        name = time.strftime('run_%Y%m%d_%H%M%S.csv')  # makes a time-stamped filename for the log
        with open(name, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['tag_id', 'stop', 'time', 'x', 'y',
                                                   'cam_x', 'cam_y', 'cam_z'])
            writer.writeheader()
            writer.writerows(self.log.values())
        self.get_logger().info(f'saved {name}')
        raise SystemExit  # ends the program


def main():
    """Run the tag logger."""
    rclpy.init()
    try:
        rclpy.spin(Scanner())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
