#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import enum
import math
import numpy
import ast


# Defining Bot States
class BotState(enum.Enum):
    LOOK_TOWARDS = 0  # rotate bots towards the goal
    GOAL_SEEK = 1  # follow line
    WALL_FOLLOW = 2  # Go around the wall / avoid obstacles


yaw = 0
yaw_threshold = math.pi / 90
goal_distance_threshold = 0.25
currentBotState = BotState.LOOK_TOWARDS

# base scan laser range values
maxRange = 3
minRange = 0

bot_pose = None
init_bot_pose = []
beacon_pose = None
bot_motion = None  # cmd_vel publisher
homing_signal = None  # subscriber
init_config_complete = False
wall_hit_point = None
beacon_found = False
twist = Twist()
distance_moved = 0

front_obs_distance = None
left_obs_distance = None

wall_folllowing = False

# goalx = rospy.get_param('goalx')
# goaly = rospy.get_param('goaly')
mapMatrix = None


class Block():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        return self.position == other.position


def translation(map_to_simulation, row, column):
    if map_to_simulation:
        return {"y": 10 - row, "x": column - 9}
    else:
        # Here row and column value are actually x and y
        return {"row": 10 - row, "column": column + 9}


def process_map():
    global mapMatrix
    f = open("./map.txt", "r").read().strip().split("map = ", 1)[1]
    f = (ast.literal_eval(f))
    mapMatrix = numpy.reshape(f, (20, 18))
    print(mapMatrix)

    # ToDo: Find A* path

    start_pos = (0, 0)
    goal_pos = (7, 8)
    path = a_star_path(start_pos, goal_pos)
    print(path)


def a_star_path(start_pos, goal_pos):
    global mapMatrix
    open_pos_list = []
    closed_pos_list = []

    # (f = g + h) //The idea here

    start_block = Block(None, start_pos)
    start_block.g = start_block.f = start_block.h = 0
    open_pos_list.append(start_block)

    goal_block = Block(None, goal_pos)
    goal_block.g = goal_block.f = goal_block.h = 0

    while len(open_pos_list) > 0:
        current_block = open_pos_list[0]
        current_index = 0

        for index, item in enumerate(open_pos_list):
            if item.f < current_block.f:
                current_block = item
                current_index = index

        # Pop current off open list, add to closed list
        open_pos_list.pop(current_index)
        closed_pos_list.append(current_block)

        # Found the goal
        if current_block == goal_block:
            path = []
            current = current_block
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # Adjacent squares

            # Get node position
            node_position = (current_block.position[0] + new_position[0], current_block.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(mapMatrix) - 1) or node_position[0] < 0 or node_position[1] > (
                    len(mapMatrix[len(mapMatrix) - 1]) - 1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if mapMatrix[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Block(current_block, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_pos_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_block.g + 1
            child.h = ((child.position[0] - goal_block.position[0]) ** 2) + (
                        (child.position[1] - goal_block.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_pos_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_pos_list.append(child)


def normalize(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def look_towards(des_pos):
    global yaw, yaw_threshold, bot_motion, currentBotState, twist
    quaternion = (
        des_pos.orientation.x,
        des_pos.orientation.y,
        des_pos.orientation.z,
        des_pos.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]  # bot's yaw
    beacon_yaw = math.atan2(goaly - des_pos.position.y, goalx - des_pos.position.x)
    yaw_diff = normalize(beacon_yaw - yaw)

    if math.fabs(yaw_diff) > yaw_threshold:
        twist.angular.z = -0.5  # clockwise rotation if yaw_diff > 0 else 0.5  # counter-clockwise rotation

    if math.fabs(yaw_diff) <= yaw_threshold:
        twist.angular.z = 0
    bot_motion.publish(twist)


def get_base_truth(bot_data):
    global bot_pose, beacon_found, goal_distance_threshold
    bot_pose = bot_data.pose.pose

    goal_distance = math.sqrt(pow(bot_pose.position.y - goaly, 2) + pow(bot_pose.position.x - goalx, 2))
    # print(goal_distance)
    if goal_distance <= goal_distance_threshold:
        beacon_found = True


def process_sensor_info(data):
    global maxRange, minRange, front_obs_distance, left_obs_distance, zone_R, zone_FR, zone_F, zone_FL, zone_L
    maxRange = data.range_max
    minRange = data.range_min

    # Note: Configuration one
    # zone = numpy.array_split(numpy.array(data.ranges), 5)
    # zone_R = zone[0]
    # zone_FR = zone[1]
    # zone_F = zone[2]
    # zone_FL = zone[3]
    # zone_L = zone[4]
    # if front_obs_distance is None and left_obs_distance is None:
    #     front_obs_distance = 0.75
    #     left_obs_distance = 2

    # Note: Configuration 2 - Breaking at uneven angles
    zone = numpy.array(data.ranges)
    zone_R = zone[0:50]  # 30deg
    zone_FR = zone[51:140]
    zone_F = zone[141:220]
    zone_FL = zone[221:310]
    zone_L = zone[311:361]
    if front_obs_distance is None and left_obs_distance is None:
        front_obs_distance = 1
        left_obs_distance = 1


def init():
    global homing_signal
    rospy.init_node("bot", anonymous=True)
    # rospy.Subscriber('/base_scan', LaserScan, process_sensor_info)
    # rospy.Subscriber('/base_pose_ground_truth', Odometry, get_base_truth)

    rospy.spin()


if __name__ == '__main__':
    try:
        process_map()
        init()
    except rospy.ROSInterruptException:
        pass



