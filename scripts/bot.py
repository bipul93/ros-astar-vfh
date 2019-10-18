#!/usr/bin/env python
from functools import reduce
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
import math
import numpy

yaw = 0
yaw_threshold = math.pi / 90
goal_distance_threshold = 0.5

bot_pose = None  # bot's position
init_bot_pos = None
bot_motion_pub = None  # cmd_vel publisher
laser_scan_sub = None  # subscriber
ground_truth_sub = None  # subscriber

goal_found = False  # final goal

goalx = rospy.get_param('goalx')
goaly = rospy.get_param('goaly')
twist = Twist()

mapMatrix = None
step_path = []  # Global path
step_path_index = 0
bot_ready = False
path_ready = False
translation_threshold = 0.25


map_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
            0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
            0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
            0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1]


# A* node class
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


# VFH Histogram class - methods to build polar histogram and to return best angle to move towards
class Histogram:
    def __init__(self, bin_width):
        self.bin_width = bin_width
        self.bin_size = 360 / bin_width
        self.histogram = [0] * self.bin_size
        self.previous_selected = None
        self.a = 2
        self.b = 1
        self.c = 1

    def __set__(self, instance, value):
        self.previous_selected = value

    def build_histogram(self, sensor_data, bot_pos, bot_yaw, step_goal, step_goal_index):

        # histogram_data = []
        # for index, item in enumerate(sensor_data):
        #     histogram_data.append(math.fabs(item-3))
        # print(histogram_data)

        # sensor_data_length = len(sensor_data) 360 degrees. taking static constant
        # obstacle condition. within 1 from range(0-3)
        openings = []
        for bin_index in range(self.bin_size):
            if bin_index == 0:
                start_index = 0
                end_index = self.bin_width
            else:
                start_index = (360 / self.bin_size) * bin_index + 1
                end_index = (360 / self.bin_size) * (bin_index + 1) + 1

            self.histogram[bin_index] = reduce(lambda count, i: count + (i < 1), sensor_data[start_index:end_index], 0)
            filter_threshold = 4
            if self.histogram[bin_index] < filter_threshold:  # Filter threshold
                # TODO: Check is bot will fit into the opening or not ?
                if bin_index-2 > 0 and bin_index+2 < self.bin_size:
                    if self.histogram[bin_index+1] > filter_threshold and self.histogram[bin_index+2] > filter_threshold:
                        continue
                openings.append(bin_index)

        # print(self.histogram)
        # print(openings)

        cost = 0
        lowest_cost_bin = None
        lowest_cost_bin_index = None
        goal_dir = math.atan2(step_goal[step_goal_index][1] - bot_pos.y, step_goal[step_goal_index][0] - bot_pos.x)
        for index, candidate_index in enumerate(openings):
            candidate_dir = normalize(bot_yaw + ((((candidate_index * self.bin_width + self.bin_width/2 ) - 180) / 2) * 0.01745))
            target_dir = math.fabs(candidate_dir - goal_dir)
            current_dir = math.fabs(candidate_dir - bot_yaw)

            if self.previous_selected is None:
                self.previous_selected = bot_yaw
            previous_dir = math.fabs(self.previous_selected - candidate_dir)

            g_cost = (self.a * target_dir + self.b * current_dir + self.c * previous_dir)

            if index == 0:
                cost = g_cost
                lowest_cost_bin = candidate_dir
                lowest_cost_bin_index = candidate_index
            elif g_cost < cost:
                cost = g_cost
                lowest_cost_bin = candidate_dir
                lowest_cost_bin_index = candidate_index
        #
        # print(lowest_cost_bin, lowest_cost_bin_index, (lowest_cost_bin * 57.2958))
        return lowest_cost_bin  #* 0.0174533


histogram = Histogram(24)


def translation(map_to_simulation, row, column):
    global translation_threshold
    if map_to_simulation:
        return column - 9 + 0.35, 10 - row - 0.35  # adding threshold to center of unit cell
    else:
        # Here row and column value are actually x and y
        return 10 - row, column + 9


def process_map():
    global mapMatrix, map_data, step_path, laser_scan_sub, path_ready
    # Todo: read file in roslaunch not working but working in rosrun
    # f = open("map.txt", "r").read().strip().split("map = ", 1)[1]
    # f = (ast.literal_eval(f))
    # mapMatrix = numpy.reshape(f, (20, 18))
    mapMatrix = numpy.reshape(map_data, (20, 18))
    # print(mapMatrix)

    # Find A* path
    # print(init_bot_pos.x, init_bot_pos.y)
    initx = init_bot_pos.x
    inity = init_bot_pos.y

    start_pos = translation(False, int(math.floor(inity)), int(math.floor(initx)))
    goal_pos = translation(False, int(math.floor(goaly)), int(math.floor(goalx)))
    print(start_pos)
    print(goal_pos)
    step_path = Astar().a_star_path(start_pos, goal_pos)

    if step_path is not False:
        print(step_path)

        for index, point in enumerate(step_path):
            step_path[index] = translation(True, point[0], point[1])
        print(step_path)
        path_ready = True
        laser_scan_sub = rospy.Subscriber('/base_scan', LaserScan, process_sensor_info)
    else:
        print("Unreachable Goal: Path not found")


class Astar:

    def heuristic(self, start, goal):
        return ((start[0] - goal[0]) ** 2) + ((start[1] - goal[1]) ** 2)

    def neighbours(self, node):
        global mapMatrix
        neighbours_list = []
        neighbour_nodes = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 8-way movement
        # neighbour_nodes = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # 4-way movement
        for index, neighbour in enumerate(neighbour_nodes, start=1):
            neighbour_pos = (node.position[0] + neighbour[0], node.position[1] + neighbour[1])

            if neighbour_pos[0] > (len(mapMatrix) - 1) \
                    or neighbour_pos[0] < 0 \
                    or neighbour_pos[1] > (len(mapMatrix[len(mapMatrix) - 1]) - 1) \
                    or neighbour_pos[1] < 0:
                continue

            if mapMatrix[neighbour_pos[0]][neighbour_pos[1]] != 0:
                continue

            # Doesn't require in 4 way movement
            if index > 4:
                # Check if adjacent node to diagonal nodes aren't 1
                if index == 5:
                    if mapMatrix[node.position[0] + neighbour_nodes[0][0]][
                        node.position[1] + neighbour_nodes[0][1]] == 1 and \
                            mapMatrix[node.position[0] + neighbour_nodes[2][0]][
                                node.position[1] + neighbour_nodes[2][1]] == 1:
                        continue
                elif index == 6:
                    if mapMatrix[node.position[0] + neighbour_nodes[2][0]][
                        node.position[1] + neighbour_nodes[2][1]] == 1 and \
                            mapMatrix[node.position[0] + neighbour_nodes[1][0]][
                                node.position[1] + neighbour_nodes[1][1]] == 1:
                        continue
                elif index == 7:
                    if mapMatrix[node.position[0] + neighbour_nodes[0][0]][
                        node.position[1] + neighbour_nodes[0][1]] == 1 and \
                            mapMatrix[node.position[0] + neighbour_nodes[3][0]][
                                node.position[1] + neighbour_nodes[3][1]] == 1:
                        continue
                elif index == 8:
                    if mapMatrix[node.position[0] + neighbour_nodes[3][0]][
                        node.position[1] + neighbour_nodes[3][1]] == 1 and \
                            mapMatrix[node.position[0] + neighbour_nodes[1][0]][
                                node.position[1] + neighbour_nodes[1][1]] == 1:
                        continue
                neighbour_node = Node(None, neighbour_pos)
                neighbour_node.g = node.g + 1.414  # diagonal_cost

            else:
                neighbour_node = Node(None, neighbour_pos)
                neighbour_node.g = node.g + 1  # normal cost

            neighbour_node.parent = node
            neighbours_list.append(neighbour_node)
        return neighbours_list

    def a_star_path(self, start_pos, goal_pos):
        open_list = []
        closed_list = []

        start_node = Node(None, start_pos)
        start_node.g = 0
        start_node.f = start_node.g + self.heuristic(start_pos, goal_pos)
        open_list.append(start_node)

        goal_node = Node(None, goal_pos)
        goal_node.g = 0
        goal_node.f = goal_node.g + self.heuristic(start_pos, goal_pos)

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
                while path_node.parent is not None:
                    path.append(path_node.position)
                    path_node = path_node.parent
                return path[::-1]

            open_list.pop(current_list_index)
            closed_list.append(current_node)

            neighbour_nodes = self.neighbours(current_node)
            for neighbour in neighbour_nodes:
                if neighbour not in closed_list:
                    neighbour.f = neighbour.g + self.heuristic(neighbour.position, goal_pos)

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


def get_base_truth(bot_data):
    global bot_pose, goal_found, goal_distance_threshold, step_path_index, step_path, bot_ready, init_bot_pos, yaw
    bot_pose = bot_data.pose.pose
    if init_bot_pos is None:
        init_bot_pos = bot_pose.position
        process_map()
        bot_ready = True

    quaternion = (
        bot_pose.orientation.x,
        bot_pose.orientation.y,
        bot_pose.orientation.z,
        bot_pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]  # bot's yaw

    if path_ready:
        goal_distance = math.sqrt(pow(bot_pose.position.y - goaly, 2) + pow(bot_pose.position.x - goalx, 2))

        if goal_distance <= goal_distance_threshold:
            goal_found = True

        step_goal_distance = math.sqrt(pow(bot_pose.position.y - step_path[step_path_index][1], 2) + pow(
            bot_pose.position.x - step_path[step_path_index][0], 2))
        if step_goal_distance <= 0.5 and step_path_index < (len(step_path) - 1):
            step_path_index += 1


def process_sensor_info(data):
    global yaw, laser_scan_sub, ground_truth_sub
    if not goal_found:
        if bot_ready and path_ready:
            # if not once:
            chosen_path = histogram.build_histogram(data.ranges, bot_pose.position, yaw, step_path, step_path_index)
            if chosen_path is not None:
                histogram.previous_selected = chosen_path  # * 57.2958
                yaw_diff = normalize(chosen_path - yaw)
                if math.fabs(yaw_diff) > yaw_threshold:
                    twist.linear.x = 0
                    twist.angular.z = -0.5 if yaw_diff < 0 else 0.5  # counter-clockwise rotation
                else:
                    twist.angular.z = 0
                    twist.linear.x = 0.5
    else:
        twist.angular.z = 0
        twist.linear.x = 0
        print(" <----------------------------------------- Reached Goal --------------------------------------------------->")
        laser_scan_sub.unregister()
        ground_truth_sub.unregister()
    bot_motion_pub.publish(twist)


def init():
    global bot_motion_pub, bot_pose, twist, ground_truth_sub
    rospy.init_node("bot", anonymous=True)
    # rospy.Subscriber('/base_scan', LaserScan, process_sensor_info)
    ground_truth_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, get_base_truth)
    bot_motion_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
