"""Loop: drive forward a bit -> stop -> look at a few frames -> log tags -> repeat.
Stops early if the laser sees an obstacle in front of the robot."""
import csv
import math
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pupil_apriltags import Detector
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

IMAGE_TOPIC = "/camera/image_raw"
ODOM_TOPIC = "/odom"
CMD_TOPIC = "/cmd_vel"
SCAN_TOPIC = "/scan"
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_data.yaml")
TAG_SIZE = 0.107      # meters: side length of the black square of tags
NUM_STOPS = 5         # how many places to stop and scan
STEP_DIST = 0.30      # meters to drive between stops
SPEED = 0.08          # m/s
WAIT_TIME = 1.0       # seconds to sit still before scanning
SCAN_FRAMES = 8       # frames to look at per stop
STOP_DIST = 0.20      # meters: stop if anything in front is closer than this
FRONT_ANGLE = 15      # degrees on each side of straight ahead to check


class Scanner(Node):
    def __init__(self):
        super().__init__("scanner")
        self.bridge = CvBridge()
        self.detector = Detector(families="tag25h9")

        # load camera calibration from the yaml file
        with open(CALIB_FILE) as f:
            calib = yaml.safe_load(f)
        self.K = np.array(calib["camera_matrix"]["data"]).reshape(3, 3)   # intrinsics matrix
        self.D = np.array(calib["distortion_coefficients"]["data"])       # lens distortion
        # pupil_apriltags needs -> (fx, fy, cx, cy)
        self.cam_params = (self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2])

        self.create_subscription(Image, IMAGE_TOPIC, self.on_image, 1)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_timer(0.1, self.loop)

        self.x = self.y = None      # robot position from odometry
        self.front_dist = None      # closest valid laser reading in front of the robot
        self.state = "scan_wait"
        self.stop = 0               # which stop we are at
        self.start_xy = None        # where the current drive began
        self.wait_start = time.time()
        self.frames = 0
        self.seen = {}              # tag_id -> position of tag in camera frame at this stop
        self.log = {}               # tag_id -> row

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def on_scan(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))   # wrap to [-pi, pi]
        front = ranges[np.abs(angles) <= math.radians(FRONT_ANGLE)]
        # throw out invalid readings (0.0, inf, nan)
        front = front[np.isfinite(front) & (front > msg.range_min) & (front < msg.range_max)]
        self.front_dist = float(front.min()) if front.size else None

    def blocked(self):
        return self.front_dist is not None and self.front_dist <= STOP_DIST

    def on_image(self, msg):
        if self.state != "scan":
            return
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, "bgr8"), cv2.COLOR_BGR2GRAY) #makes the image grayscale and converts to opencv image
        gray = cv2.undistort(gray, self.K, self.D)  # removes lens distortion using the calibration
        # detect tags and estimate their 3D pose relative to the camera
        for tag in self.detector.detect(gray, estimate_tag_pose=True,
                                        camera_params=self.cam_params, tag_size=TAG_SIZE):
            self.seen[tag.tag_id] = tag.pose_t.flatten()  # [x, y, z] in meters, z is forward
        self.frames += 1

    #simple function to move the robot at a given speed
    def move(self, speed=0.1):
        cmd = Twist()
        cmd.linear.x = speed
        self.pub.publish(cmd)

    #main state machine
    def loop(self):
        if self.x is None:
            return  # no odometry yet

        if self.state == "drive":
            driven = math.hypot(self.x - self.start_xy[0], self.y - self.start_xy[1]) #euclidean distance between where this drive began vs where the robot is now
            if self.blocked():  # obstacle ahead: stop early and scan from here
                self.get_logger().warn(f"obstacle at {self.front_dist:.2f} m, stopping early")
                self.move(0.0)
                self.state = "scan_wait"
                self.wait_start = time.time()
            elif driven >= STEP_DIST: # if this dist is greater than the step, stop and wait before scanning
                self.move(0.0)
                self.state = "scan_wait"
                self.wait_start = time.time()
            else:
                self.move(SPEED) #if not at the step dist yet, keep driving

        elif self.state == "scan_wait": #stop and wait WAIT_TIME seconds, then switch to SCAN state
            self.move(0.0)
            if time.time() - self.wait_start >= WAIT_TIME:
                self.frames = 0
                self.seen = {}
                self.state = "scan"
                self.get_logger().info(f"stop {self.stop}: scanning for tags")

        elif self.state == "scan": #wait until it gets SCAN_FRAMES frames, then saves the tag and either finishes the run or moves to the next stop
            if self.frames >= SCAN_FRAMES:
                self.save_stop()
                self.stop += 1
                if self.stop >= NUM_STOPS:
                    self.finish()
                    self.get_logger().info("finished run")
                elif self.blocked():  # can't drive further, so end the run
                    self.get_logger().warn("path blocked, ending run early")
                    self.finish()
                else:
                    self.start_xy = (self.x, self.y)
                    self.state = "drive"
                    self.get_logger().info(f"driving to stop {self.stop}")

    def save_stop(self):
        for tag_id, t in self.seen.items():
            if tag_id not in self.log:  # ignore duplicates
                #adds tag id to the log with its ID, stop number, time, x/y coordinates, and camera-frame position
                self.log[tag_id] = {"tag_id": tag_id, "stop": self.stop,
                                    "time": time.strftime("%H:%M:%S"),
                                    "x": round(self.x, 2), "y": round(self.y, 2),
                                    "cam_x": round(float(t[0]), 3),
                                    "cam_y": round(float(t[1]), 3),
                                    "cam_z": round(float(t[2]), 3)}
                self.get_logger().info(f"stop {self.stop}: found tag {tag_id} at {t[2]:.2f} m")

    def finish(self):
        self.move(0.0)
        name = time.strftime("run_%Y%m%d_%H%M%S.csv") #makes a time-stamped filename for the log
        with open(name, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["tag_id", "stop", "time", "x", "y",
                                                   "cam_x", "cam_y", "cam_z"])
            writer.writeheader()
            writer.writerows(self.log.values())
        self.get_logger().info(f"saved {name}")
        raise SystemExit  # ends the program


def main():
    rclpy.init()
    try:
        rclpy.spin(Scanner())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()