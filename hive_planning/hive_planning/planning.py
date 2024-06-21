#!/usr/bin/env python3

import time
import math
import rclpy
import cv2 as cv
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Int16MultiArray, Int64MultiArray, Int8
from std_msgs.msg import Int8
from hive_planning.d_star_lite import DStarLite
from hive_planning.grid import OccupancyGridMap, SLAM

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from std_msgs.msg import Float32MultiArray

import random
from PIL import Image
import matplotlib.pyplot as plt
from math import pi


UNOCCUPIED = 0
OBSTACLE = 255

n = 1
X_DIM = 640 / 4
Y_DIM = 480 / 4
MAX_LIN_VEL = 1
MAX_ANG_VEL = 1
MAX_STEP = 10

serving_flag = False
FREE_STATE = int(1)
INIT_STATE = int(2)
WEBMOVE_STATE = int(3)
DOCKING_STATE = int(4)
TABLE_STATE = int(5)
SERVING_STATE = int(6)
PARKING_STATE = int(7)
totalRobotcnt = int(3)
REAL_X = 2.57
REAL_Y = 1.94
batch_size = 4
fixelConst_x = REAL_X / (X_DIM)  # Pixel x length in meters
fixelConst_y = REAL_Y / (Y_DIM)  # Pixel y length in meters
spread_distance = 8
backward_time = 30
backward_List = []

flag = False

x_target = 0
y_target = 0
yaw_target = 0


def quaternion_to_euler_angle(quaternion):
    """
    Convert Quaternion to Euler angles (yaw, pitch, roll)
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return yaw_z, pitch_y, roll_x


class Path_Planner():
    def __init__(self):
        self.map = OccupancyGridMap(int(X_DIM), int(Y_DIM))
        self.first = True
        self.myNum = 1
        self.initPose = (0, 0)
        self.goal = (0, 0)
        self.way = []
        self.dstar = DStarLite(map=self.map, s_start=self.initPose, s_goal=self.goal)
        self.slam = SLAM(map=self.map, view_range=int(2 * 2))

    def map_cb(self, input, height, width):
        print("in the cb")

        h = width  # height and width are swapped
        w = height

        batch = int(4)

        # Reduce map size
        input = input[:(w // batch) * batch, :(h // batch) * batch].reshape(w // batch, batch, h // batch, batch).max(axis=(1, 3)).astype(int)

        # Update and compare map
        for j in range(int(w / batch)):
            for i in range(int(h / batch)):
                if input[j, i] > 0:
                    input[j, i] = OBSTACLE
                else:
                    input[j, i] = UNOCCUPIED

        return input

    def planning(self, curr_pose):
        self.curr_pose = curr_pose
        self.slam.set_ground_truth_map(gt_map=self.map)

        new_edges_and_old_costs, slam_map = self.slam.rescan(g_pos_1=self.curr_pose)
        self.dstar.new_edges_and_old_costs = new_edges_and_old_costs
        self.dstar.sensed_map = slam_map

        self.way, g, rhs = self.dstar.move_and_replan(robot_position=self.curr_pose)

        if self.way:
            return self.way


class PlannerNode(Node):
    def __init__(self):
        super().__init__('PathPlanner_node')
        self.get_logger().info('Start planner')

        self.planner = Path_Planner()

        self.declare_parameter('robot_num', 1)

        self.MYNUM = self.get_parameter('robot_num').get_parameter_value().integer_value
        self.get_logger().info('Robot number: %d' % self.MYNUM)

        self.rx = []
        self.ry = []

        self.mode = 0
        self.initPose_List = (tuple(np.round([77, 27]).astype(int)), tuple(np.round([24, 27]).astype(int)), tuple(np.round([130, 27]).astype(int)))
        self.initPose = self.initPose_List[self.MYNUM - 1]

        self.spreadPose = (self.initPose[0], self.initPose[1] + spread_distance)
        self.get_logger().info('spreadPose :' + str(self.spreadPose))

        self.map = OccupancyGridMap(int(X_DIM), int(Y_DIM))
        self.p_map = [[0 for _ in range(int(Y_DIM))] for _ in range(int(X_DIM))]
        self.serve_btn = 0
        self.target_yaw = 0.0
        self.way_point = 0
        self.arrived = 0
        self.kitchen = (4, 0)
        self.setter_flag = True
        self.serving_flag = 1
        self.docking_flag = 0
        self.parking_flag = False
        self.curr_pose = self.initPose

        self.tableGoal_xy = [0, 0]

        self.check_yaw = 0.0
        self.curr_yaw = 0.0

        qos_profile = rclpy.qos.QoSProfile(depth=10)
        qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE

        # Subscribers
        self.plannerGoal_sub = self.create_subscription(Int64MultiArray, '/plannerGoal', self.plannerGoal_callback, 10)
        self.dockingGoal_sub = self.create_subscription(Int16MultiArray, '/dockingGoal', self.dockingGoal_callback, 10)
        self.servingSignal_sub = self.create_subscription(Int64MultiArray, '/servingGoal', self.servingSignal_callback, 10)
        self.parkingSignal_sub = self.create_subscription(Int64MultiArray, '/parkingGoal', self.parkingSignal_callback, 10)

        self.odom_subscription = self.create_subscription(Odometry, '/robot' + str(self.MYNUM) + '/final_pose', self.odom_callback, 10)
        # self.gridOdom_subscription = self.create_subscription(Int16MultiArray, '/robot' + str(self.MYNUM) + '/grid_odom', self.gridOdom_callback, 10)

        self.serveBtn_subscriber = self.create_subscription(Int8, '/robot' + str(self.MYNUM) + '/serve_btn', self.serveBTN_callback, 10)
        # self.gridMap_sub = self.create_subscription(OccupancyGrid,'/robot' + str(self.MYNUM) + '/occupancyGridmap',self.gridMap_callback,qos_profile)
        self.robotState_publisher = self.create_publisher(Int16MultiArray, '/stateFromplanner', 10)

        self.backward_vel_publisher = self.create_publisher(Int16MultiArray, '/robot' + str(self.MYNUM) + '/backward_vel', 10)
        self.dockingPostion_publisher = self.create_publisher(String, '/robot' + str(self.MYNUM) + '/docking_position', 10)
        self.targetPostion_publisher = self.create_publisher(Int16MultiArray, '/robot' + str(self.MYNUM) + '/target_position', 10)
        self.goal_status_subscriber = self.create_subscription(Int16MultiArray, '/robot' + str(self.MYNUM) + '/goal_status', self.goal_status_callback, 10)
        self.timer = self.create_timer(0.5, self.run)

    def plannerGoal_callback(self, msgs):
        # Planner goal callback
        plannerGoal = msgs.data
        self.get_logger().info('Planner msgs    :' + str(plannerGoal))

        if plannerGoal[0] == self.MYNUM:
            self.setter_flag = True
            self.mode = plannerGoal[1]
            self.goalFrom_master = plannerGoal[2:-1]
            self.target_yaw = plannerGoal[-1]

            self.goalFrom_master = np.array(self.goalFrom_master, dtype=np.uint8)
            self.goalFrom_master = tuple(self.goalFrom_master.astype(int))

            if self.mode == DOCKING_STATE:
                self.docking_flag = 1

        # Determine target yaw
        if (self.target_yaw == 1):
            self.check_yaw = 0.0
        elif (self.target_yaw == -1):
            self.check_yaw = -180
        elif (self.target_yaw == 2):
            self.check_yaw = -180 / 2
        elif (self.target_yaw == -2):
            self.check_yaw = 180 / 2
        self.check_yaw = math.radians(self.check_yaw)

    def dockingGoal_callback(self, msgs):
        # Docking goal callback
        dockingGoal = msgs.data
        self.get_logger().info('Docking msgs    :' + str(dockingGoal))
        if dockingGoal[0] == self.MYNUM:
            self.dockingGoal_xy = dockingGoal[1:-1]
            self.dockingGoal_yaw = dockingGoal[-1]

    def servingSignal_callback(self, msgs):
        # Serving signal callback
        servingGoal = msgs.data
        if servingGoal[0] == self.MYNUM:
            self.setter_flag = True
            self.mode = servingGoal[1]
            self.servegoal = servingGoal[2:4]
            self.serve_yaw = servingGoal[-1]
            self.get_logger().info('msgs    :' + str(self.servegoal))
        self.get_logger().info('servingGoal sub!!')

    def serveBTN_callback(self, msgs):
        # Serve button callback
        self.serve_btn = 1
        self.get_logger().info('serve_btn:  ' + str(self.serve_btn))
        self.get_logger().info('servingBTN sub!!')

    def parkingSignal_callback(self, msgs):
        # Parking signal callback
        self.get_logger().info('parking sub!!')
        self.get_logger().info(str(msgs.data[0]))
        self.get_logger().info(str(msgs.data[1]))

        if self.MYNUM == msgs.data[0]:
            self.get_logger().info('Now robot parking state!!')
            self.setter_flag = True
            self.mode = msgs.data[1]
            self.parking_flag = True

    def odom_callback(self, msgs):
        """
        '/robot1/odom' topic callback. Updates and controls the current position and orientation.
        """
        # Odometry is in world coordinates, so x and y need to be swapped to match the planning coordinate system
        self.curr_pose = tuple(np.round([msgs.pose.pose.position.x / fixelConst_x, msgs.pose.pose.position.y / fixelConst_y]).astype(int))
        self.curr_ori = msgs.pose.pose.orientation
        self.curr_yaw, _, _ = quaternion_to_euler_angle(self.curr_ori)

    def gridOdom_callback(self, msgs):
        # Placeholder for grid odometry callback
        pass

    def gridMap_callback(self, msgs):
        # Placeholder for grid map callback
        try:
            pass
        except Exception as e:
            self.get_logger().error(f'Error in gridMap_callback: {e}')

    def goal_status_callback(self, msg):
        # Goal status callback
        goal_status_parts = msg.data.split(',')
        goal_arrived_flag = bool(int(goal_status_parts[0]))  # goal_status_parts[0] is arrival status, goal_arrived_flag is true if arrived
        self.way_point = int(goal_status_parts[1])  # goal_status_parts[1] is self.way_point value

    # UTIL
    def setterFunction(self, curr_pose, goal):
        # Function to set planner goal
        if self.setter_flag:
            self.planner.dstar = DStarLite(map=self.map, s_start=curr_pose, s_goal=goal)
            self.way_point = 0

    def distanceCheckFuntion(self, curr_pose, goal, tolerance):
        # Function to check if distance to goal is within tolerance
        distance = math.sqrt((goal[0] - curr_pose[0]) ** 2 + (goal[1] - curr_pose[1]) ** 2)
        if distance <= tolerance:
            return 1
        else:
            return 0

    def run(self):
        # Main run loop
        if self.mode == FREE_STATE:
            pass
        elif self.mode == TABLE_STATE:
            pass
        elif self.mode == INIT_STATE:
            spreadPosen = self.curr_pose

            self.setterFunction(self.curr_pose, spreadPosen)
            self.get_logger().info("curr_pose, spreadPose" + str(self.curr_pose) + ", " + str(self.spreadPose))

            if self.distanceCheckFuntion(self.curr_pose, spreadPosen, 4.0):
                self.get_logger().info("Publish MOVE STATE")
                robot_state_msg = Int16MultiArray()
                robot_state_msg.data = [self.MYNUM, WEBMOVE_STATE]
                self.robotState_publisher.publish(robot_state_msg)

            self.planner.planning(self.curr_pose)
            self.control(self.planner.way)

        elif self.mode == WEBMOVE_STATE:
            self.dockingPose = self.goalFrom_master

            self.setterFunction(self.curr_pose, self.dockingPose)
            self.planner.planning(self.curr_pose)

            yaw_error = abs(self.check_yaw - self.curr_yaw)
            if yaw_error > pi * 2:
                yaw_error = 0.0

            if self.arrived and self.distanceCheckFuntion(self.dockingPose, self.curr_pose, 5.6) and yaw_error < 0.035:
                self.get_logger().info("Publish DOCKING STATE")
                robot_state_msg = Int16MultiArray()
                robot_state_msg.data = [self.MYNUM, DOCKING_STATE]
                self.mode = DOCKING_STATE
                self.planner.way = []
                self.robotState_publisher.publish(robot_state_msg)

            else:
                self.get_logger().info("check_yaw: " + str(abs(yaw_error)))

            self.control(self.planner.way)

        elif self.mode == DOCKING_STATE:
            if self.docking_flag:
                self.get_logger().info("Docking STATE Run")
                self.tableGoal_xy = self.dockingGoal_xy
                self.tableGoal_yaw = self.dockingGoal_yaw
                docking_msg = String()
                docking_msg.data = str(self.tableGoal_xy[0]) + ',' + str(self.tableGoal_xy[1]) + ',' + str(self.tableGoal_yaw)
                self.get_logger().info("Publish,,,")
                self.dockingPostion_publisher.publish(docking_msg)
                self.docking_flag = 0

            self.get_logger().info("curr_pose " + str(self.curr_pose[0]) + ' ' + str(self.curr_pose[1]))
            self.get_logger().info("tableGoal_xy " + str(self.tableGoal_xy[0]) + ' ' + str(self.tableGoal_xy[1]))

            if self.distanceCheckFuntion(self.tableGoal_xy, self.curr_pose, 1.5):
                self.get_logger().info("Publish TABLE STATE")
                robot_state_msg = Int16MultiArray()
                robot_state_msg.data = [self.MYNUM, TABLE_STATE]
                self.mode = TABLE_STATE
                self.robotState_publisher.publish(robot_state_msg)

        elif self.mode == SERVING_STATE:
            if self.serving_flag == 1:
                self.get_logger().info("SERVING_STATE")
                self.kitchen_goal = self.curr_pose
                self.setterFunction(self.curr_pose, self.kitchen_goal)

            if self.distanceCheckFuntion(self.servegoal, self.curr_pose, 5.6) and self.serve_btn:
                self.get_logger().info("MOVE initPose Goal")
                self.serve_btn = 0
                self.setter_flag = True
                self.serving_flag = 3
                self.mode = PARKING_STATE
                self.parking_flag = False
                self.target_yaw = -1
                self.setterFunction(self.curr_pose, self.spreadPose)

            if (self.distanceCheckFuntion(self.kitchen_goal, self.curr_pose, 10.5) and self.serve_btn) or self.serving_flag == 2:
                self.get_logger().info("MOVE Serving Goal")
                self.get_logger().info('msgs    :' + str(self.curr_pose) + "  " + str(self.servegoal))
                self.serving_flag = 2
                self.serve_btn = 0
                self.setter_flag = True
                self.target_yaw = -1 * int(self.serve_yaw)
                self.setterFunction(self.curr_pose, tuple(self.servegoal))

                self.planner.planning(self.curr_pose)
                self.get_logger().info("serve planning,,,")
                self.control(self.planner.way)

        elif self.mode == PARKING_STATE:
            self.get_logger().info("PARKING_STATE")
            if self.parking_flag:
                self.get_logger().info("BackWord publish,,,")
                self.parking_flag = False
                self.target_yaw = -1
                backward_vel_msg = Int16MultiArray()
                backward_List = []
                backward_List.append(int(backward_time))
                backward_List.append(int(self.initPose[0]))
                backward_List.append(int(self.initPose[1]))
                backward_vel_msg.data = backward_List
                self.backward_vel_publisher.publish(backward_vel_msg)
                self.planner.planning(self.curr_pose)

            else:
                self.setterFunction(self.curr_pose, self.spreadPose)

                if self.distanceCheckFuntion(self.initPose, self.curr_pose, 3.0):
                    self.get_logger().info("Publish FREE_STATE STATE")
                    robot_state_msg = Int16MultiArray()
                    robot_state_msg.data = [self.MYNUM, FREE_STATE]
                    self.mode = FREE_STATE
                    self.robotState_publisher.publish(robot_state_msg)

                elif not (self.distanceCheckFuntion(self.spreadPose, self.curr_pose, 5.6)):
                    self.get_logger().info("first spread")
                    self.planner.planning(self.curr_pose)
                    self.control(self.planner.way)

                else:
                    if abs(self.curr_yaw) < 0.035:
                        self.get_logger().info("Now Spread pose")
                        self.get_logger().info("back,,,")
                        backward_vel_msg = Int16MultiArray()
                        backward_List = []
                        backward_List.append(int(0))
                        backward_List.append(int(self.initPose[0]))
                        backward_List.append(int(self.initPose[1]))
                        self.get_logger().info("backward LIST: " + str(backward_List[0]) + ", " + str(backward_List[1]) + ", " + str(backward_List[2]))

                        backward_vel_msg.data = backward_List
                        self.backward_vel_publisher.publish(backward_vel_msg)
                        self.get_logger().info("publish,,,")

                    data = []
                    self.arrived = 1
                    self.target_yaw = 1
                    data.append(int(self.target_yaw))
                    data.append(int(self.arrived))
                    target_msg = Int16MultiArray()
                    target_msg.data = data
                    self.targetPostion_publisher.publish(target_msg)

    def control(self, way):
        # Control the robot using the planned waypoints
        data = []
        self.get_logger().info('curr    :' + str(self.curr_pose))

        if not isinstance(way, list) or not all(isinstance(w, tuple) and len(w) == 2 for w in way):
            self.get_logger().error('Invalid way format: ' + str(way))
            return

        if self.way_point + 4 < len(way):
            for i in range(0, 4, 1):
                waypoint = way[self.way_point + i]
                if isinstance(waypoint, tuple) and len(waypoint) == 2:
                    data.append(int(waypoint[0]))
                    data.append(int(waypoint[1]))
                else:
                    self.get_logger().error('Invalid waypoint: ' + str(waypoint))
                    return
            self.arrived = 0
            self.get_logger().info('way    :' + str(way))
        else:
            self.arrived = 1
            self.get_logger().info("Reached the end of the path. No more targets to publish.")

        data.append(int(self.target_yaw))
        data.append(int(self.arrived))

        target_msg = Int16MultiArray()
        target_msg.data = data
        self.targetPostion_publisher.publish(target_msg)

def main(args=None):
    while True:
        try:
            rclpy.init(args=args)
            node = PlannerNode()
            node.get_logger().info('==== Planning node Start! ====')
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.destroy_node()
            rclpy.shutdown()
            node.get_logger().info('==== Server stopped cleanly ====')
            break
        except BaseException as e:
            node.get_logger().error(f'Exception in server: {repr(e)}')
            node.get_logger().info('!! Exception in server: Restarting node')
            continue
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()