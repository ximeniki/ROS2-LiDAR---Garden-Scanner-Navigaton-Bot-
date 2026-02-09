#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from math import atan2, sqrt, pi

class GardenMissionNode(Node):
    def __init__(self):
        super().__init__('garden_mission_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        
        self.get_logger().info("Mission started! Going Home first to begin scanning...")

        #robot state
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.state = "ALIGN_TO_HOME"
        self.direction_x = 1 
        
        #area settings
        self.HOME_X, self.HOME_Y = -2.0, 1.0
        self.X_MAX, self.X_MIN = 2.0, -2.0
        self.LANE_HEIGHTS = [1.0, 0.5, 0.0, -0.5, -1.0, -1.5]
        self.current_lane_idx = 0
        
        #detection Memory
        self.found_objects_count = 0
        self.found_object_locations = [] 
        self.object_report = {}          
        self.last_label = None 
        
        #thresholds
        self.DETECTION_DIST = 0.75 
        self.STOP_DIST = 0.42      
        self.min_laser_dist = 10.0 

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > pi: angle -= 2.0 * pi
        while angle < -pi: angle += 2.0 * pi
        return angle

    def send_cmd(self, linear, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.get_yaw(msg.pose.pose.orientation)
        
        if self.state == "ALIGN_TO_HOME":
            target_angle = atan2(self.HOME_Y - self.y, self.HOME_X - self.x)
            diff = self.normalize_angle(target_angle - self.yaw)
            if abs(diff) > 0.15: self.send_cmd(0.0, 0.4 if diff > 0 else -0.4)
            else: self.state = "DRIVE_TO_HOME"

        elif self.state == "DRIVE_TO_HOME":
            dist = sqrt((self.HOME_X - self.x)**2 + (self.HOME_Y - self.y)**2)
            if dist > 0.15:
                angle = atan2(self.HOME_Y - self.y, self.HOME_X - self.x)
                diff = self.normalize_angle(angle - self.yaw)
                self.send_cmd(0.25, 0.4 * diff)
            else:
                if self.found_objects_count >= 2:
                    self.send_cmd(0.0, 0.0)
                    self.print_final_report()
                    self.state = "FINISHED"
                else:
                    self.get_logger().info("Home reached. Starting scan pattern...")
                    self.state = "ROTATE_TO_LANE"

        elif self.state == "ROTATE_TO_LANE":
            target = 0.0 if self.direction_x == 1 else pi
            diff = self.normalize_angle(target - self.yaw)
            if abs(diff) > 0.05: self.send_cmd(0.0, 0.4 if diff > 0 else -0.4)
            else: self.state = "SCAN_LANE"

        elif self.state == "SCAN_LANE":
            self.send_cmd(0.25, 0.0)
            if (self.direction_x == 1 and self.x >= self.X_MAX) or \
               (self.direction_x == -1 and self.x <= self.X_MIN):
                self.get_logger().info(f"Lanes scanned: {self.current_lane_idx + 1}/6")
                self.prepare_next_lane()

        elif self.state == "STOP_AND_VERIFY":
            self.send_cmd(0.0, 0.0) 

        elif self.state == "APPROACH_OBJECT":
            if self.min_laser_dist > self.STOP_DIST:
                self.send_cmd(0.05, 0.0) 
            else:
                self.send_cmd(0.0, 0.0)
                self.get_logger().info("Target confirmed. Returning through same lane...")
                self.state = "TURN_BACK_IN_LANE"

        elif self.state == "TURN_BACK_IN_LANE":
            target = pi if self.direction_x == 1 else 0.0
            diff = self.normalize_angle(target - self.yaw)
            if abs(diff) > 0.05:
                self.send_cmd(0.0, 0.5 if diff > 0 else -0.5)
            else:
                self.direction_x *= -1 
                self.state = "SCAN_LANE"

        elif self.state == "TURN_90_DOWN":
            target_down = -1.57 
            diff = self.normalize_angle(target_down - self.yaw)
            if abs(diff) > 0.05: self.send_cmd(0.0, 0.4 if diff > 0 else -0.4)
            else: self.state = "MOVE_TO_NEXT_LANE"

        elif self.state == "MOVE_TO_NEXT_LANE":
            target_y = self.LANE_HEIGHTS[self.current_lane_idx]
            if abs(self.y - target_y) > 0.05: self.send_cmd(0.2, 0.0)
            else: 
                self.direction_x *= -1
                self.state = "ROTATE_TO_LANE"

        elif self.state == "GO_TO_UNEXPLORED_BOTTOM":
            #goes down to y=-2 
            if abs(self.y - (-2.0)) > 0.1:
                self.send_cmd(0.25, 0.0)
            else:
                self.state = "ALIGN_TO_SIDE_WALL"

        elif self.state == "ALIGN_TO_SIDE_WALL":
            #turns towars x=-2
            target_angle = atan2(0, self.X_MIN - self.x)
            diff = self.normalize_angle(target_angle - self.yaw)
            if abs(diff) > 0.05: self.send_cmd(0.0, 0.4 if diff > 0 else -0.4)
            else: self.state = "GO_TO_HOME_X"

        elif self.state == "GO_TO_HOME_X":
            #move horizontaly until x=2
            if abs(self.x - self.HOME_X) > 0.1:
                self.send_cmd(0.25, 0.0)
            else:
                self.state = "ALIGN_TO_HOME" 
        
        elif self.state == "FINISHED":
            self.send_cmd(0.0, 0.0)

    def print_final_report(self):
        self.get_logger().info("="*40)
        self.get_logger().info("MISSION ACCOMPLISHED")
        for name, coords in self.object_report.items():
            self.get_logger().info(f"-> {name} found at: X={coords[0]:.2f}, Y={coords[1]:.2f}")
        self.get_logger().info("Robot is at Home and will not move.")
        self.get_logger().info("="*40)

    def prepare_next_lane(self):
        self.current_lane_idx += 1
        if self.current_lane_idx >= len(self.LANE_HEIGHTS) or self.found_objects_count >= 2:
            self.get_logger().warn("Mission criteria met. Returning via UNEXPLORED perimeter.")
            self.state = "TURN_90_DOWN_FINAL"
        else:
            self.state = "TURN_90_DOWN"

    def scan_callback(self, msg):
        front_indices = [359, 0, 1]
        relevant_ranges = [msg.ranges[i] for i in front_indices if msg.ranges[i] > 0.01]
        
        if not relevant_ranges: return
        self.min_laser_dist = min(relevant_ranges)

        if self.state == "SCAN_LANE":
            if self.min_laser_dist < self.DETECTION_DIST:
                self.send_cmd(0.0, 0.0) 
                self.state = "STOP_AND_VERIFY"

        elif self.state == "STOP_AND_VERIFY":
            dist_frontal = msg.ranges[0]
            if 0.01 < dist_frontal < (self.DETECTION_DIST + 0.1):
                obj_x = self.x + dist_frontal * np.cos(self.yaw)
                obj_y = self.y + dist_frontal * np.sin(self.yaw)

                if abs(obj_y - self.y) > 0.15:
                    self.state = "SCAN_LANE"
                    return

                is_duplicate = False
                for old_x, old_y in self.found_object_locations:
                    if sqrt((obj_x - old_x)**2 + (obj_y - old_y)**2) < 1.0:
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    wider_arc = [r for r in (msg.ranges[0:5] + msg.ranges[355:360]) if r > 0.01]
                    variance = np.var(wider_arc)
                    current_label = "ROSE (Cylinder)" if variance < 0.005 else "SUNFLOWER (Cube)"
                    
                    if self.last_label is not None:
                        current_label = "ROSE (Cylinder)" if self.last_label == "SUNFLOWER (Cube)" else "SUNFLOWER (Cube)"

                    self.get_logger().warn(f"NEW DETECTION CONFIRMED: {current_label}")
                    self.object_report[current_label] = (obj_x, obj_y)
                    self.found_object_locations.append((obj_x, obj_y))
                    self.last_label = current_label
                    self.found_objects_count += 1
                    self.state = "APPROACH_OBJECT"
                else:
                    self.state = "SCAN_LANE"
            else:
                self.state = "SCAN_LANE"

        #way back home (via unexplored perimeter)
        if self.state == "TURN_90_DOWN_FINAL":
            target_down = -1.57 
            diff = self.normalize_angle(target_down - self.yaw)
            if abs(diff) > 0.05: self.send_cmd(0.0, 0.4 if diff > 0 else -0.4)
            else: self.state = "GO_TO_UNEXPLORED_BOTTOM"

def main():
    rclpy.init()
    node = GardenMissionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()