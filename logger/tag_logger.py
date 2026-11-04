"""Loop: drive forward a bit -> stop -> look at a few frames -> log tags -> repeat
Run:  python3 scan_node.py"""
import csv
import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pupil_apriltags import Detector
from rclpy.node import Node
from sensor_msgs.msg import Image

IMAGE_TOPIC = "/camera/image_raw"
ODOM_TOPIC = "/odom"
CMD_TOPIC = "/cmd_vel"
NUM_STOPS = 5        # how many places to stop and scan
STEP_DIST = 0.30     # meters to drive between stops
SPEED = 0.08         # m/s
WAIT_TIME = 1.0      # seconds to sit still before scanning
SCAN_FRAMES = 8      # frames to look at per stop


class Scanner(Node):
    def __init__(self):
        super().__init__("scanner")
        self.bridge = CvBridge()
        self.detector = Detector(families="tag36h11")
        self.create_subscription(Image, IMAGE_TOPIC, self.on_image, 1)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_timer(0.1, self.loop)

        self.x = self.y = None      # robot position from odometry
        self.state = "scan_wait"    
        self.stop = 0               # which stop we are at
        self.start_xy = None        # where the current drive began
        self.wait_start = time.time()
        self.frames = 0
        self.seen = set()           # tag IDs seen at this stop
        self.log = {}               # tag_id -> row

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def on_image(self, msg):
        if self.state != "scan":
            return
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, "bgr8"), cv2.COLOR_BGR2GRAY) #makes the image grayscale and converts to opencv image
        for tag in self.detector.detect(gray):            # adds the tag id to the set of seen tags at this stop
            self.seen.add(tag.tag_id)
        self.frames += 1

    #simple function to move the robot at a given speed
    def move(self, speed): 
        cmd = Twist()
        cmd.linear.x = speed
        self.pub.publish(cmd)

    #main state machine 
    def loop(self):
        if self.x is None:
            return  # no odometry yet

        if self.state == "drive":
            driven = math.hypot(self.x - self.start_xy[0], self.y - self.start_xy[1]) #euclidean distance between where this drive began vs where the robot is now
            if driven >= STEP_DIST: # if this dist is greater than the step, stop and wait before scanning
                self.move(0.0)
                self.state = "scan_wait"
                self.wait_start = time.time()
            else:
                self.move(SPEED) #if not at the step dist yet, keep driving

        elif self.state == "scan_wait": #stop and wait WAIT_TIME seconds, then switch to SCAN state
            self.move(0.0)
            if time.time() - self.wait_start >= WAIT_TIME: 
                self.frames = 0
                self.seen = set()
                self.state = "scan"

        elif self.state == "scan": #wait until it gets SCAN_FRAMES frames, then saves the tag and either finishes the run or moves to the next stop
            if self.frames >= SCAN_FRAMES: 
                self.save_stop()
                self.stop += 1
                if self.stop >= NUM_STOPS:
                    self.finish()
                else:
                    self.start_xy = (self.x, self.y)
                    self.state = "drive"

    def save_stop(self):
        for tag_id in self.seen:
            if tag_id not in self.log:  # ignore duplicates
                #adds tag id to the log with its ID, stop number, time, and x/y coordinates
                self.log[tag_id] = {"tag_id": tag_id, "stop": self.stop,
                                    "time": time.strftime("%H:%M:%S"),
                                    "x": round(self.x, 2), "y": round(self.y, 2)}
                self.get_logger().info(f"stop {self.stop}: found tag {tag_id}")

    def finish(self):
        name = time.strftime("run_%Y%m%d_%H%M%S.csv") #makes a time-stamped filename for the log
        with open(name, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["tag_id", "stop", "time", "x", "y"])
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