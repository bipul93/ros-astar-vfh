#!/usr/bin/env python
from functools import reduce

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from tf.transformations import euler_from_quaternion

import enum
import math
import numpy
import ast
# from .vfh.histogram import Histogram


# Defining Bot States
class BotState(enum.Enum):
    LOOK_TOWARDS = 0  # rotate bots towards the goal
    GOAL_SEEK = 1  # follow line
    WALL_FOLLOW = 2  # Go around the wall / avoid obstacles


yaw = 0
yaw_threshold = math.pi / 90
goal_distance_threshold = 0.25

# base scan laser range values
maxRange = 3
minRange = 0

bot_pose = None
init_bot_pose = []
bot_motion = None  # cmd_vel publisher
homing_signal = None  # subscriber
init_config_complete = False
wall_hit_point = None
beacon_found = False
twist = Twist()
distance_moved = 0

front_obs_distance = None
left_obs_distance = None


goalx = rospy.get_param('goalx')
goaly = rospy.get_param('goaly')
mapMatrix = None


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.f = 0
        self.g = 0
        self.h = 0
        self.diagonal = False

    def __eq__(self, other):
        return self.position == other.position


class Histogram:
    def __init__(self, bin_size):
        self.bin_size = bin_size
        self.bin_width = 360 / bin_size
        self.histogram = [0] * bin_size

    def build_histogram(self, sensor_data):
        smoothed_histogram = [0] * self.bin_size
        # histogram_data = []
        # for index, item in enumerate(sensor_data):
        #     histogram_data.append(math.fabs(item-3))
        # print(histogram_data)

        # sensor_data_length = len(sensor_data) 360 degrees. taking static constant
        # obstacle condition. within 1 from range(0-3)
        for bin_index in range(self.bin_size):
            if bin_index == 0:
                start_index = 0
                end_index = 73
            else:
                start_index = (360/self.bin_size)*bin_index + 1
                end_index = (360/self.bin_size)*(bin_index + 1) + 1

            self.histogram[bin_index] = reduce(lambda count, i: count + (i < 2), sensor_data[start_index:end_index], 0)
            print(start_index, end_index)
            print(sensor_data[start_index:end_index])
        print(self.histogram)


histogram = Histogram(5)


def translation(simulation_to_map, row, column):
    if simulation_to_map:
        return (10-row, column-9)
    else:
        # Here row and column value are actually x and y
        # return {"row": 10 - row, "column": column + 9}
        return (10 - row, column + 9)


def process_map():
    global mapMatrix
    f = open("./map.txt", "r").read().strip().split("map = ", 1)[1]
    f = (ast.literal_eval(f))
    mapMatrix = numpy.reshape(f, (20, 18))
    print(mapMatrix)

    # ToDo: Find A* path
    initx = -8.0
    inity = -2.0

    goalx = 4.0
    goaly = 9.0

    start_pos = translation(False, int(math.floor(inity)), int(math.floor(initx)))
    goal_pos = translation(False, int(math.floor(goaly)), int(math.floor(goalx)))
    print(start_pos)
    print(goal_pos)
    path = a_star_path(start_pos, goal_pos)
    print(path)

    # for point in range(len(path)):
    #     print(mapMatrix[path[point][0]][path[point][1]])


def heuristic(start, goal):
    return ((start[0] - goal[0]) ** 2) + ((start[1] - goal[1]) ** 2)


def neighbours(node):
    global mapMatrix
    neighbours_list = []
    # neighbour_nodes = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 8-way movement
    neighbour_nodes = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # 4-way movement
    for index, neighbour in enumerate(neighbour_nodes, start=1):
        neighbour_pos = (node.position[0] + neighbour[0], node.position[1] + neighbour[1])

        if neighbour_pos[0] > (len(mapMatrix) - 1) \
                or neighbour_pos[0] < 0 \
                or neighbour_pos[1] > (len(mapMatrix[len(mapMatrix) - 1]) - 1) \
                or neighbour_pos[1] < 0:
            continue

        if mapMatrix[neighbour_pos[0]][neighbour_pos[1]] != 0:
            continue

        neighbour_node = Node(None, neighbour_pos)
        if index > 4:
            neighbour_node.g = node.g + 1.414  # diagonal_cost
        else:
            neighbour_node.g = node.g + 1  # normal cost
        neighbour_node.parent = node
        neighbours_list.append(neighbour_node)
    return neighbours_list


def a_star_path(start_pos, goal_pos):
    open_list = []
    closed_list = []

    start_node = Node(None, start_pos)
    start_node.g = 0
    start_node.f = start_node.g + heuristic(start_pos, goal_pos)
    open_list.append(start_node)

    goal_node = Node(None, goal_pos)
    goal_node.g = 0
    goal_node.f = goal_node.g + heuristic(start_pos, goal_pos)

    path = []
    while len(open_list) > 0:
        current_node = open_list[0]
        current_list_index = 0

        # open list element with lowest f score
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_list_index = index

        # Goal found
        if current_node == goal_node:
            path_node = current_node
            print(path_node.parent)
            while path_node.parent is not None:
                path.append(path_node.position)
                path_node = path_node.parent
            return path[::-1]

        open_list.pop(current_list_index)
        closed_list.append(current_node)

        neighbour_nodes = neighbours(current_node)
        for neighbour in neighbour_nodes:
            if neighbour not in closed_list:
                neighbour.f = neighbour.g + heuristic(neighbour.position, goal_pos)

                if neighbour not in open_list:
                    open_list.append(neighbour)
                else:
                    open_neighbour = open_list[open_list.index(neighbour)]
                    if neighbour.g < open_neighbour.g:
                        open_neighbour.g = neighbour.g
                        open_neighbour.parent = neighbour.parent
    return False


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
    # print(data.ranges)
    histogram.build_histogram(data.ranges)
    global maxRange, minRange, front_obs_distance, left_obs_distance, zone_R, zone_FR, zone_F, zone_FL, zone_L
    maxRange = data.range_max
    minRange = data.range_min



def init():
    rospy.init_node("bot", anonymous=True)
    rospy.Subscriber('/base_scan', LaserScan, process_sensor_info)
    # rospy.Subscriber('/base_pose_ground_truth', Odometry, get_base_truth)

    rospy.spin()


if __name__ == '__main__':
    try:
        process_map()
        # init()
    except rospy.ROSInterruptException:
        pass



