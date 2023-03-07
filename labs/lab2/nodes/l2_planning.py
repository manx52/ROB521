#!/usr/bin/env python3
# Standard Libraries
import numpy as np
import yaml
import pygame
import time
import pygame_utils
import matplotlib.image as mpimg
from skimage.draw import circle_perimeter
from scipy.linalg import block_diag
import scipy.spatial as sp
import matplotlib.pyplot as plt
import math

COLORS = dict(
    w=(255, 255, 255),
    k=(0, 0, 0),
    g=(0, 255, 0),
    r=(255, 0, 0),
    b=(0, 0, 255)
)


# Map Handling Functions
def load_map(filename):
    im = mpimg.imread("../maps/" + filename)
    im_np = np.array(im)  # Whitespace is true, black is false
    # im_np = np.logical_not(im_np)
    return im_np


def load_map_yaml(filename):
    with open("../maps/" + filename, "r") as stream:
        map_settings_dict = yaml.safe_load(stream)
    return map_settings_dict


# Node for building a graph
class Node:
    def __init__(self, point, parent_id, cost, id):
        self.id = id
        self.point = point  # A 3 by 1 vector [x, y, theta]
        self.parent_id = parent_id  # The parent node id that leads to this node (There should only every be one parent in RRT)
        self.cost = cost  # The cost to come to this node
        self.children_ids = []  # The children node ids of this node
        self.traj = []
        return


# Path Planner
class PathPlanner:
    # A path planner capable of perfomring RRT and RRT*
    def __init__(self, map_filename, map_setings_filename, goal_point, stopping_dist):
        # Get map information
        self.occupancy_map = load_map(map_filename)
        self.map_shape = self.occupancy_map.shape
        self.map_settings_dict = load_map_yaml(map_setings_filename)

        # Get the metric bounds of the map
        self.bounds = np.zeros([2, 2])  # m
        self.bounds[0, 0] = self.map_settings_dict["origin"][0]
        self.bounds[1, 0] = self.map_settings_dict["origin"][1]
        self.bounds[0, 1] = self.map_settings_dict["origin"][0] + self.map_shape[1] * self.map_settings_dict[
            "resolution"]
        self.bounds[1, 1] = self.map_settings_dict["origin"][1] + self.map_shape[0] * self.map_settings_dict[
            "resolution"]

        self.bounds_pixel = np.zeros([2, 2])  # m
        self.bounds_pixel[0, 0] = self.map_settings_dict["origin"][0]
        self.bounds_pixel[1, 0] = self.map_settings_dict["origin"][1]
        self.bounds_pixel[0, 1] = self.map_settings_dict["origin"][0] + self.map_shape[0] * self.map_settings_dict[
            "resolution"]
        self.bounds_pixel[1, 1] = self.map_settings_dict["origin"][1] + self.map_shape[1] * self.map_settings_dict[
            "resolution"]

        # for i in range(self.map_shape[0]):
        #     for j in range(self.map_shape[1]):
        #         self.occupancy_map[i,0] = 0
        #         self.occupancy_map[i, self.map_shape[1] - 1] = 0
        #         self.occupancy_map[0, j] = 0
        #         self.occupancy_map[self.map_shape[0] - 1, j] = 0

        # Robot information
        self.robot_radius = 0.22  # m
        self.vel_max = 1  # m/s (Feel free to change!)
        self.rot_vel_max = 1  # rad/s (Feel free to change!)

        # Goal Parameters
        self.goal_point = goal_point  # m
        self.stopping_dist = stopping_dist  # m
        self.flag = 0

        # Trajectory Simulation Parameters
        self.timestep = 1.0  # s
        self.num_substeps = 10

        # Planning storage
        self.nodes = [Node(np.zeros((3, 1)), -1, 0, 0)]

        # RRT* Specific Parameters
        self.lebesgue_free = np.sum(self.occupancy_map) * self.map_settings_dict["resolution"] ** 2
        self.zeta_d = np.pi
        self.gamma_RRT_star = 2 * (1 + 1 / 2) ** (1 / 2) * (self.lebesgue_free / self.zeta_d) ** (1 / 2)
        self.gamma_RRT = self.gamma_RRT_star + .1
        self.epsilon = 2.5
        self.alpha = 0.4

        # Pygame window for visualization
        self.window = pygame_utils.PygameWindow(
            "Path Planner", (1000, 1000), self.occupancy_map.shape, self.map_settings_dict, self.goal_point,
            self.stopping_dist)
        return

    # Functions required for RRT
    def sample_map_space(self):
        # Return an [x,y] coordinate to drive the robot towards
        # print("TO DO: Sample point to drive towards")

        sample = np.zeros((2, 1))
        # real_bounds = np.array([[-3.5, 43.5], [-49.25, 20]])
        real_bounds = self.bounds  # np.array([[0.0, 43.5], [-46, 10]])
        real_bounds = np.array([[0.0, 43.5], [-46, 10]])

        sample[0] = np.random.uniform(low=real_bounds[0, 0], high=real_bounds[0, 1])
        sample[1] = np.random.uniform(low=real_bounds[1, 0], high=real_bounds[1, 1])
        # return sample

        if abs(self.dist_to_goal(self.nodes[-1].point)) < 20:
            self.flag = 1
        if self.flag == 1:
            scale = 1.3 * self.dist_to_goal(self.nodes[-1].point)
            scale_x = scale  # * (real_bounds[0,1] - real_bounds[0,0])  # 10% box
            scale_y = scale  # * (real_bounds[1,1] - real_bounds[1,0])

            curr_coord = self.goal_point  # self.nodes[-1].point
            x_box_low = np.clip(curr_coord[0] - scale_x, real_bounds[0, 0], real_bounds[0, 1])
            y_box_low = np.clip(curr_coord[1] - scale_y, real_bounds[1, 0], real_bounds[1, 1])

            x_box_high = np.clip(curr_coord[0] + scale_x, real_bounds[0, 0], real_bounds[0, 1])
            y_box_high = np.clip(curr_coord[1] + scale_y, real_bounds[1, 0], real_bounds[1, 1])

            x_sample = np.random.uniform(low=x_box_low, high=x_box_high)
            y_sample = np.random.uniform(low=y_box_low, high=y_box_high)

            sample[0] = x_sample
            sample[1] = y_sample
        # sample[0]=input("x")
        # sample[1]=input("y")
        # self.window.add_point(np.array(sample).reshape(2,), radius=1, width=0,
        #                        color=COLORS['b'])
        return sample

    def check_if_duplicate(self, point):
        # Check if point is a duplicate of an already existing node
        duplicate_range = 0.1
        for i in self.nodes:
            # print(np.linalg.norm(i.point-point),i.point.reshape(3,),point)
            if np.linalg.norm(i.point.reshape(3, ) - point) <= duplicate_range:
                return True
        return False

    def check_colision(self, robot_traj):
        # pass in trajectory = 3 X N from simulate_trajectory() and check for collision
        # False -> collision detected
        # True -> no collision
        traj_rr, traj_cc = self.points_to_robot_circle(
            robot_traj[0:2, :])  # center and radius of trajectory in occupacy map
        footprint = np.moveaxis(np.array([traj_rr, traj_cc]), 0, 2)
        temp_x = np.clip(footprint[..., 1], 0, self.map_shape[0] - 1)
        temp_y = np.clip(footprint[..., 0], 0, self.map_shape[1] - 1)

        return np.any(np.any(self.occupancy_map[temp_x, temp_y] == 0, axis=-1))

    def closest_node(self, point, n):
        # print(point)
        # Returns the index of the closest node

        kdtree = sp.cKDTree(np.stack([node.point[:2, :] for node in self.nodes], axis=0).squeeze(-1))
        d, i = kdtree.query(point.T, k=n)
        # print("here: ", np.array(i).shape)
        # print(i)
        # filter inf
        i = np.array(i)
        if len(i.shape) > 1:
            i = list(filter(lambda a: a != len(self.nodes), i[0]))

            if len(i) < n:
                return i

        return i[0:n]

    def simulate_trajectory(self, node_i, point_s):
        # Simulates the non-holonomic motion of the robot.
        # This function drives the robot from node_i towards point_s. This function does has many solutions!
        # node_i is a 3 by 1 vector [x;y;theta] this can be used to construct the SE(2) matrix T_{OI} in course notation
        # point_s is the sampled point vector [x; y]

        # print("TO DO: Implment a method to simulate a trajectory given a sampled point")
        vel, rot_vel = self.robot_controller2(node_i, point_s)

        if abs(self.dist_to_goal(node_i) > 1):  # 10 for sim
            traj = self.trajectory_rollout2(vel, rot_vel, node_i[0], node_i[1], node_i[2])
        else:

            min_dist_goal = float("inf")
            traj = None
            for i in range(1, 11):
                if i <= 5:
                    robot_traj = self.trajectory_rollout2(vel, rot_vel + (i * 0.025), node_i[0], node_i[1], node_i[2])
                else:
                    robot_traj = self.trajectory_rollout2(vel, rot_vel - ((i - 5) * 0.025), node_i[0], node_i[1],
                                                          node_i[2])
                dist_to_goal = self.dist_to_goal(robot_traj[:, -1])

                if traj is None:
                    min_dist_goal = dist_to_goal
                    traj = robot_traj
                else:
                    collision = self.check_colision(robot_traj)
                    duplicate = self.check_if_duplicate(robot_traj[:, -1])
                    if not (collision or duplicate):
                        if dist_to_goal < min_dist_goal:
                            min_dist_goal = dist_to_goal
                            traj = robot_traj

        return traj

    def dist_to_goal(self, point):
        # goal distance
        x_g = self.goal_point[0] - point[0]
        y_g = self.goal_point[1] - point[1]

        return np.sqrt(x_g ** 2 + y_g ** 2)

    def robot_controller2(self, node_i, point_s):
        # This controller determines the velocities that will nominally move the robot from node i to node s
        # Max velocities should be enforced
        # alpha is the tuning parameter
        theta_d = np.arctan2((point_s[1] - node_i[1]), (point_s[0] - node_i[0]))
        theta = node_i[2]
        heading_error = theta_d - theta
        heading_error_norm = math.atan2(math.sin(heading_error), math.cos(heading_error))
        # print(heading_error)
        rot_vel = -4 * self.alpha * np.tan(heading_error_norm)

        if rot_vel > self.rot_vel_max:
            rot_vel = self.rot_vel_max
        if rot_vel < -self.rot_vel_max:
            rot_vel = -self.rot_vel_max

        vel = self.vel_max / (abs(rot_vel) + 1)  # not sure

        if vel > self.vel_max:
            vel = self.vel_max
        if vel < -self.vel_max:
            vel = -self.vel_max

        if rot_vel > vel / self.robot_radius:
            rot_vel = vel / self.robot_radius

        return vel, -rot_vel

    def trajectory_rollout2(self, vel=0, rot_vel=0, x0=0, y0=0, theta0=0):
        # Given your chosen velocities determine the trajectory of the robot for your given timestep
        # The returned trajectory should be a series of points to check for collisions
        # re_turn 3 x n
        t = np.linspace(0, self.timestep, self.num_substeps)
        x0 = np.ones((1, self.num_substeps)) * x0
        y0 = np.ones((1, self.num_substeps)) * y0
        theta0 = np.ones((1, self.num_substeps)) * theta0
        if rot_vel == 0:
            x = vel * t * np.cos(theta0) + x0
            y = vel * t * np.sin(theta0) + y0
            theta = np.zeros_like(t)
        else:
            # temp_x = (vel / rot_vel) * (np.sin(rot_vel * t + theta0) - np.sin(theta0)) + x0
            # offset = 0.5
            # x = np.clip(temp_x, self.bounds[0,0] + offset, self.bounds[0,1]-offset)
            # temp_y = -(vel / rot_vel) * (np.cos(rot_vel * t + theta0) - np.cos(theta0)) + y0
            # y = np.clip(temp_y, self.bounds[1, 0]+offset, self.bounds[1, 1] -offset)
            # if np.any(np.any(temp_x == x, axis=-1)) or np.any(np.any(temp_y == y, axis=-1)):
            #     theta = theta0
            # else:
            #     theta = (rot_vel * t + theta0) % (2 * math.pi)
            x = (vel / rot_vel) * (np.sin(rot_vel * t + theta0) - np.sin(theta0)) + x0
            # offset = 0.5
            # x = np.clip(x, self.bounds[0, 0] + offset, self.bounds[0, 1] - offset)
            y = -(vel / rot_vel) * (np.cos(rot_vel * t + theta0) - np.cos(theta0)) + y0
            # y = np.clip(y, self.bounds[1, 0] + offset, self.bounds[1, 1] - offset)
            theta = (rot_vel * t + theta0) % (2 * math.pi)
        est_trajectory = np.vstack((x, y, theta))

        return est_trajectory

    def point_to_cell(self, point):
        # Convert a series of [x,y] points in the map to the indices for the corresponding cell in the occupancy map
        # point is a 2 by N matrix of points of interest

        # print("TO DO: Implement a method to get the map cell the robot is currently occupying")

        new_point = point.copy()
        new_point[0] = (point[0] - self.bounds[0, 0]) // self.map_settings_dict["resolution"]
        new_point[1] = self.map_shape[0] - (point[1] - self.bounds[1, 0]) // self.map_settings_dict["resolution"]
        new_point = (np.floor(new_point)).astype(int)
        return new_point

    def points_to_robot_circle(self, points):
        # Convert a series of [x,y] points to robot map footprints for collision detection
        # Hint: The disk function is included to help you with this function

        # print("TO DO: Implement a method to get the pixel locations of the robot path")

        rr = []
        cc = []
        robot_radius = np.floor(self.robot_radius // self.map_settings_dict["resolution"]).astype(int)

        for pt in (np.transpose(points)):
            occ = self.point_to_cell(pt)

            temp_rr, temp_cc = circle_perimeter(occ[0], occ[1], robot_radius)

            rr.append(temp_rr)
            cc.append(temp_cc)
        return np.array(rr), np.array(cc)

    # RRT* specific functions
    def ball_radius(self):
        # Close neighbor distance
        card_V = len(self.nodes)
        return min(self.gamma_RRT * (np.log(card_V) / card_V) ** (1.0 / 2.0), self.epsilon)

    def connect_node_to_point(self, node_i, point_f):
        # Given two nodes find the non-holonomic path that connects them
        # Settings
        # node is a 3 by 1 node
        # point is a 2 by 1 point
        # line 1 through start tangent to theta
        # line 2 normal to line 1
        # line 3 through start and final
        # line 4 through midpoint prependicular to line 3

        start = node_i.reshape((3,))[0:2]
        theta_d = np.arctan2((point_f[1] - node_i[1]), (point_f[0] - node_i[0]))
        theta = node_i.reshape((3,))[2]
        heading_error = theta_d - theta

        final = point_f.reshape((2,))
        dist = np.linalg.norm(start - final)
        string_mid_point = (final - start) / 2 + start
        s1 = np.tan(theta)
        if s1 == 0:
            s2 = 9998
        else:
            s2 = -1 / s1
        s3d = (final[0] - start[0]) + 0.001
        s3 = (final[1] - start[1]) / s3d
        if s3 == 0:
            s4 = 9999
        else:
            s4 = -1 / s3
        y1 = start[1] - s1 * start[0]
        y2 = start[1] - s2 * start[0]
        y3 = start[1] - s3 * start[0]
        y4 = string_mid_point[1] - s4 * string_mid_point[0]
        # self.window.add_point([string_mid_point[0], string_mid_point[0] * s4 + y4], radius=3, width=0,
        #                       color=COLORS['b'])

        # plot line 1

        # self.window.add_line(start,[start[0]+10,(start[0]+10)*s1+y1])
        # self.window.add_line(start,[start[0]+10,(start[0]+10)*s2+y2])
        # self.window.add_line(start,[start[0]+10,(start[0]+10)*s3+y3])
        # self.window.add_line(string_mid_point,[string_mid_point[0]+10,(string_mid_point[0]+10)*s4+y4])

        cx = (y4 - y2) / (s2 - s4)
        cy = cx * s2 + y2
        center = np.array([cx, cy])
        r = np.linalg.norm(start - center)
        # print(r,np.linalg.norm(final-center))

        c = start
        b = center
        a = final

        ba = a - b
        bc = c - b

        # cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        # angle = np.arccos(cosine_angle)
        angle = -(np.arctan2(start[1] - center[1], start[0] - center[0]) - (
            np.arctan2(final[1] - center[1], final[0] - center[0])))
        st = np.sin(-theta)
        ct = np.cos(-theta)
        R = np.array([[ct, -st], [st, ct]])
        c_R = R @ center.reshape((2, 1)) + start.reshape((2, 1))
        # print(c_R, angle,"check")
        if c_R[1, 0] < 0:
            if angle > 0:
                angle = -2 * math.pi + angle
            else:
                angle = angle

        else:
            if angle < 0:
                angle = 2 * math.pi + angle
            else:
                angle = angle

        length = abs(angle) * r
        # print(length,angle)
        ignore_vel_max = True
        if ignore_vel_max:
            num_step = self.num_substeps
            rot_vel = angle / self.timestep
            vel = length / self.timestep

        else:
            num_step = self.num_substeps
            rot_vel = angle / self.timestep
            vel = length / self.timestep

            if rot_vel > self.rot_vel_max:
                rot_vel = self.rot_vel_max
            if rot_vel < -self.rot_vel_max:
                rot_vel = -self.rot_vel_max
            if vel > self.vel_max:
                vel = self.vel_max
            if vel < -self.vel_max:
                vel = -self.vel_max

            if rot_vel > vel / self.robot_radius:
                rot_vel = vel / self.robot_radius


        # print("vel",vel,"rot_vel",rot_vel)
        # self.window.add_point([cx,cy],radius=5, width=0, color=COLORS['r'])

        # self.window.add_point(final,radius=3, width=0, color=COLORS['g'])
        # rot_vel=0

        traj = self.trajectory_rollout2(vel, rot_vel, x0=start[0], y0=start[1], theta0=theta)
        # traj = self.trajectory_rollout(vel, rot_vel, node_i)
        # for i in np.transpose(traj[0:2,:]):
        #   #print(np.linalg.norm(i-center))
        # self.vis(np.transpose(traj[0:2,:]))

        return traj

    def cost_to_come(self, trajectory_o, node):
        # The cost to get to a node from lavalle

        # print("TO DO: Implement a cost to come metric")

        cost = 0.0
        # Path distance
        for i in range(1, len(trajectory_o[0])):
            x_d = trajectory_o[0, i] - trajectory_o[0, i - 1]
            y_d = trajectory_o[1, i] - trajectory_o[1, i - 1]

            theta_s = math.atan2(y_d, x_d)
            theta_d = theta_s - node[2]
            theta_d_norm = math.atan2(math.sin(theta_d), math.cos(theta_d))
            cost += np.sqrt(x_d ** 2 + y_d ** 2  )#+ (0.1* theta_d_norm)**2)

        return cost

    def update_children(self, node_id):
        # Given a node_id with a changed cost, update all connected nodes with the new cost
        # print("TO DO: Update the costs of connected nodes after rewiring.")

        curr_cost = self.nodes[node_id].cost

        for i, child in enumerate(self.nodes[node_id].children_ids):
            traj = self.connect_node_to_point(self.nodes[node_id].point,
                                              np.array(self.nodes[child].point[:2, -1].reshape((2, 1))))
            self.nodes[child].cost = self.cost_to_come(traj, self.nodes[child].point) + curr_cost

            # if len(self.nodes[child].children_ids) != 0:
            #     self.update_children(child)

    # Planner Functions
    def rrt_planning(self):
        # This function performs RRT on the given map and robot
        # You do not need to demonstrate this function to the TAs, but it is left in for you to check your work
        for i in range(50000):  # Most likely need more iterations than this to complete the map!
            # Sample map space
            point = self.sample_map_space()

            # Get the closest point
            closest_node_id = self.closest_node(point, 1)[0]
            # print("closet node", self.nodes[closest_node_id].point.reshape(3, ), "picked point", point.reshape(2, ))

            # Simulate driving the robot towards the closest point
            trajectory_o = self.simulate_trajectory(self.nodes[closest_node_id].point, point)

            # Check for collisions
            # print("TO DO: Check for collisions and add safe points to list of nodes.")
            collision = self.check_colision(trajectory_o)
            duplicate = self.check_if_duplicate(trajectory_o[:, -1])
            # print("dup", duplicate)

            # collision=False
            if not (collision or duplicate):
                # Add node to list
                cost = 0
                self.nodes[closest_node_id].children_ids.append(self.nodes[-1].id + 1)
                self.nodes.append(
                    Node(np.array(trajectory_o[:, -1].reshape((3, 1))), closest_node_id, cost, self.nodes[-1].id + 1))

                # visualize
                # print("added node", self.nodes[-1].id, self.nodes[-1].parent_id)
                temp_pt = np.array(trajectory_o[0:2, :]).copy().T
                self.window.add_se2_pose(np.array(trajectory_o[:, -1].reshape((3,))))
                self.vis(temp_pt)

                # Check if goal has been reached
                [x, y] = [trajectory_o[0, -1], trajectory_o[1, -1]]
                x_d = x - self.goal_point[0]
                y_d = y - self.goal_point[1]
                if abs(math.sqrt(x_d ** 2 + y_d ** 2)) < self.stopping_dist:
                    # print("Array: ", np.array(trajectory_o[:, -1].reshape((3, 1))))
                    # print("Array node: ", self.nodes[-1].point)
                    # print("traj: ", x, "  ", y)
                    # print("goal: ", self.goal_point[0], "  ", self.goal_point[1])
                    # print("diff: ", x - self.goal_point[0], "  ", y - self.goal_point[1])
                    # print("diff: ", (x - self.goal_point[0]) ** 2, "  ", (y - self.goal_point[1] ** 2))
                    print("rrt success")
                    return self.recover_path()
        # self.window.add_point(point.reshape(2, ))
        # print("rrt failed")
        return None

    def rrt_star_planning(self):
        # This function performs RRT* for the given map and robot
        i = 0
        while True:  # Most likely need more iterations than this to complete the map!
            # Sample
            point = self.sample_map_space()

            # Closest Node
            closest_node_id = self.closest_node(point, 1)[0]

            # Simulate trajectory
            trajectory_o = self.simulate_trajectory(self.nodes[closest_node_id].point, point)

            # Check for Collision
            collision = self.check_colision(trajectory_o)
            duplicate = self.check_if_duplicate(trajectory_o[:, -1])

            # collision=False
            if not (collision or duplicate):
                # Calculate cost
                if i > 0:
                    cost = self.cost_to_come(trajectory_o, self.nodes[closest_node_id].point) + self.nodes[
                        closest_node_id].cost
                    # return self.recover_path()

                else:
                    cost = 0

                pt = np.array(trajectory_o[:2, -1].reshape((2, 1)))

                # add node
                self.nodes[closest_node_id].children_ids.append(self.nodes[-1].id + 1)
                self.nodes.append(
                    Node(np.array(trajectory_o[:, -1].reshape((3, 1))), closest_node_id, cost, self.nodes[-1].id + 1))
                self.nodes[-1].traj = trajectory_o[:3, :].reshape((3, 10))
                N = 10
                R = 3.5
                if i > N:

                    n_closest_nodes = self.closest_node(pt, N)
                    closest_nodes = []
                    for nodes in n_closest_nodes:
                        if np.linalg.norm(self.nodes[nodes].point[0:2] - point) <= R:
                            closest_nodes.append(nodes)
                    # print(len(closest_nodes))
                    cost_min = cost
                    cost_min_id = closest_node_id
                    # print(cost)
                    for node in closest_nodes[1:]:  # ignore closest node
                        # Simulate trajectory from near nodes to pt
                        trajectory_node = self.connect_node_to_point(self.nodes[node].point, pt)

                        # Check for Collision
                        collision = self.check_colision(trajectory_node)
                        if not collision:
                            # Calculate cost
                            if i > 0:
                                temp_cost = self.cost_to_come(trajectory_node, self.nodes[node].point) + self.nodes[
                                    node].cost
                                # print(cost,temp_cost)
                            else:
                                temp_cost = 0

                            if temp_cost < cost_min:  # get min cost
                                cost_min = temp_cost
                                cost_min_id = node
                                trajectory_o = trajectory_node

                    # Rewire pt to another node
                    if cost_min_id != closest_node_id:
                        # remove parent connection
                        self.nodes[closest_node_id].children_ids.remove(self.nodes[-1].id)

                        # add node to min cost node
                        self.nodes[cost_min_id].children_ids.append(self.nodes[-1].id)
                        self.nodes[-1].cost = cost_min
                        self.nodes[-1].parent_id = cost_min_id
                        self.nodes[-1].traj = trajectory_o[:3, :].reshape((3, 10))
                        self.update_children(self.nodes[-1].id)
                        # print("rewire1")

                    # Rewire other nodes to pt
                    for node in closest_nodes:
                        # Skip if parent node
                        if self.nodes[node].id == cost_min_id:
                            continue

                        # Simulate trajectory from pt to near nodes
                        trajectory_node = self.connect_node_to_point(self.nodes[-1].point, np.array(
                            self.nodes[node].point[:2, -1].reshape((2, 1))))

                        # Check for Collision
                        collision = self.check_colision(trajectory_node)
                        if not collision:
                            # Calculate cost
                            if i > 0:
                                cost = self.cost_to_come(trajectory_node, self.nodes[-1].point) + self.nodes[-1].cost
                            else:
                                cost = 0
                            # print(cost,self.nodes[node].cost)
                            # New path has less cost. Rewire
                            if cost < self.nodes[node].cost:
                                # print("rewire2")
                                # remove parent connection
                                self.nodes[self.nodes[node].parent_id].children_ids.remove(node)
                                trajectory_old = self.connect_node_to_point(
                                    self.nodes[self.nodes[node].parent_id].point,
                                    np.array(self.nodes[node].point[:2, -1].reshape((2, 1))))

                                # add node to pt
                                self.nodes[node].cost = cost
                                self.nodes[node].parent_id = self.nodes[-1].id
                                self.nodes[node].traj = trajectory_node[:3, :].reshape((3, 10))

                                # updating children of pt
                                self.nodes[self.nodes[-1].id].children_ids.append(node)
                                self.update_children(node)

                                temp_pt = np.array(trajectory_node[0:2, :]).copy().T
                                temp_pt_old = np.array(trajectory_old[0:2, :]).copy().T
                                self.window.add_se2_pose(np.array(trajectory_node[:, -1].reshape((3,))))
                                self.vis(temp_pt)
                                # self.remove_vis(temp_pt_old)

                # visualize
                # print("added node", self.nodes[-1].id, self.nodes[-1].parent_id)
                temp_pt = np.array(trajectory_o[0:2, :]).copy().T
                self.window.add_se2_pose(np.array(trajectory_o[:, -1].reshape((3,))))
                self.vis(temp_pt)

                # visualize
                # print("added node", self.nodes[-1].id, self.nodes[-1].parent_id)
                # temp_pt = np.array(self.nodes[-1].traj[0:2, :]).copy().T
                # # self.window.add_se2_pose(np.array(trajectory_o[:, -1].reshape((3,))))
                # # self.window.add_se2_pose(np.array(trajectory_o[:, -1].reshape((3,))))
                # self.vis(temp_pt)
                # print(temp_pt)
                #

                # Check if goal has been reached
                # dist_to_goal = abs(self.dist_to_goal(trajectory_o[0:2, -1]))
                # if dist_to_goal < self.stopping_dist:
                [x, y] = [trajectory_o[0, -1], trajectory_o[1, -1]]
                x_d = x - self.goal_point[0]
                y_d = y - self.goal_point[1]
                if abs(math.sqrt(x_d ** 2 + y_d ** 2)) < self.stopping_dist:
                    # print("traj: ", x, "  ", y)
                    # print("goal: ", self.goal_point[0], "  ", self.goal_point[1])
                    # print("diff: ", x - self.goal_point[0], "  ", y - self.goal_point[1])
                    # print("diff: ", (x - self.goal_point[0]) ** 2, "  ", (y - self.goal_point[1])** 2)
                    # for node in self.nodes[1:]:
                    #     temp_pt = np.array(node.traj[0:2, :]).copy().T
                    #     self.vis(temp_pt)
                    #     self.window.add_se2_pose(np.array(node.traj[:, 0]).reshape((3,)))
                    print("rrt star success")
                    return self.recover_path()

            # self.window.add_point(point.reshape(2, ))
            # print(point.reshape(2, ))
            # print("rrt star failed")

            i += 1

    def recover_path(self, node_id=-1):
        path = [self.nodes[node_id].point]
        current_node_id = self.nodes[node_id].parent_id
        while current_node_id > -1:
            path.append(self.nodes[current_node_id].point)
            current_node_id = self.nodes[current_node_id].parent_id
            print(current_node_id)
        path.reverse()
        print("Path recovered")
        return path

    def vis(self, points):
        for i in points:
            self.window.add_point(i)
            # pass
        # self.window.display()

    def remove_vis(self, points):
        for i in points:
            self.window.add_point(i, color=COLORS['w'])


def main():
    # Set map information
    map_filename = "willowgarageworld_05res.png"
    map_setings_filename = "willowgarageworld_05res.yaml"
    # map_filename = "myhal.png"
    # map_setings_filename = "myhal.yaml"
    # robot information
    # goal_point = np.array([[3], [-10]])  # m
    # goal_point = np.array([[7], [0]])  # m
    goal_point = np.array([[42], [-44]])  # m
    stopping_dist = 0.5  # m

    # RRT precursor
    path_planner = PathPlanner(map_filename, map_setings_filename, goal_point, stopping_dist)

    # nodes = path_planner.rrt_planning()
    nodes = path_planner.rrt_star_planning()
    node_path_metric = np.hstack(path_planner.recover_path())

    # Leftover test functions
    np.save("shortest_path_rrtstar.npy", node_path_metric)
    # np.save("shortest_path_rrt.npy", node_path_metric)
    # np.save("shortest_path_rrtstar_myhal.npy", node_path_metric)

    # print(nodes)
    for i in nodes:
        print(i)
        path_planner.window.add_point(np.array(i[:2]).reshape(2, ), radius=4, width=0, color=COLORS['r'])
    path_planner.window.display()


if __name__ == '__main__':
    main()
