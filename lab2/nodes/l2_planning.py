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

        # Robot information
        self.robot_radius = 0.22  # m
        self.vel_max = 0.5  # m/s (Feel free to change!)
        self.rot_vel_max = 1  # rad/s (Feel free to change!)

        # Goal Parameters
        self.goal_point = goal_point  # m
        self.stopping_dist = stopping_dist  # m

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
        real_bounds = np.array([[-3.5, 43.5], [-49.25, 20]])
        return np.random.rand(2, 1) * (real_bounds[:, [1]] - real_bounds[:, [0]]) + real_bounds[:, [0]]

        # sample = np.zeros((2, 1))
        #
        # scale_x = 0.05 * (self.bounds[0, 1] - self.bounds[0, 0])  # 10% box
        # scale_y = 0.05 * (self.bounds[1, 1] - self.bounds[1, 0])
        #
        # curr_coord = self.nodes[-1].point
        # x_box_low = np.clip(curr_coord[0] - scale_x, self.bounds[0, 0], self.bounds[0, 1])
        # y_box_low = np.clip(curr_coord[1] - scale_y, self.bounds[1, 0], self.bounds[1, 1])
        #
        # x_box_high = np.clip(curr_coord[0] + scale_x, self.bounds[0, 0], self.bounds[0, 1])
        # y_box_high = np.clip(curr_coord[1] + scale_y, self.bounds[1, 0], self.bounds[1, 1])
        #
        # x_sample = np.random.uniform(low=x_box_low, high=x_box_high)
        # y_sample = np.random.uniform(low=y_box_low, high=y_box_high)
        #
        # sample[0] = x_sample
        # sample[1] = y_sample
        # return sample

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
        # print(self.occupancy_map[footprint[..., 1], footprint[..., 0]][np.any(self.occupancy_map[footprint[..., 1], footprint[..., 0]] == 0, axis = -1)])
        collision = np.any(np.any(self.occupancy_map[footprint[..., 1], footprint[..., 0]] == 0, axis=-1))
        return np.any(np.any(self.occupancy_map[footprint[..., 1], footprint[..., 0]] == 0, axis=-1))

    def closest_node(self, point, n):
        # print(point)
        # Returns the index of the closest node

        kdtree = sp.cKDTree(np.stack([node.point[:-1, :] for node in self.nodes], axis=0).squeeze(-1))
        d, i = kdtree.query(point.T, k=1)
        return i[0]

    def robot_controller_temp(self, node_i, point_s):
        # This controller determines the velocities that will nominally move the robot from node i to node s
        # Max velocities should be enforced
        # print("TO DO: Implement a control scheme to drive you towards the sampled point")
        x_d = point_s[0] - node_i[0]
        y_d = point_s[1] - node_i[1]

        theta_s = math.atan2(y_d, x_d)
        theta_d = theta_s - node_i[2]
        theta_d_norm = math.atan2(math.sin(theta_d), math.cos(theta_d))

        speed = math.sqrt(y_d ** 2 + x_d ** 2) / self.num_substeps

        if theta_d_norm > 1.5708:  # Behind from left side
            speed = -speed
            theta_d_norm -= 3.14
        elif theta_d_norm < -1.5708:  # Behind from right side
            speed = -speed
            theta_d_norm += 3.14

        theta_speed = theta_d_norm / self.num_substeps

        rot_vel = np.clip(theta_speed, -self.rot_vel_max, self.rot_vel_max)
        vel = np.clip(speed, -self.vel_max, self.vel_max)

        return vel, rot_vel

    def trajectory_rollout_temp(self, vel, rot_vel, node):
        # Given your chosen velocities determine the trajectory of the robot for your given timestep
        # The returned trajectory should be a series of points to check for collisions
        # print("TO DO: Implement a way to rollout the controls chosen")
        robot_pose = node
        traj = np.zeros((3, self.num_substeps))
        for i in range(self.num_substeps):
            traj[0, i] = robot_pose[0] + vel * math.cos(robot_pose[2]) * self.timestep
            traj[1, i] = robot_pose[1] + vel * math.sin(robot_pose[2]) * self.timestep
            traj[2, i] = robot_pose[2] + rot_vel * self.timestep

            robot_pose = traj[:, i]
        return traj

    def simulate_trajectory(self, node_i, point_s):
        # Simulates the non-holonomic motion of the robot.
        # This function drives the robot from node_i towards point_s. This function does has many solutions!
        # node_i is a 3 by 1 vector [x;y;theta] this can be used to construct the SE(2) matrix T_{OI} in course notation
        # point_s is the sampled point vector [x; y]

        # print("TO DO: Implment a method to simulate a trajectory given a sampled point")

        vel, rot_vel = self.robot_controller(node_i, point_s)
        robot_traj = self.trajectory_rollout(vel, rot_vel, node_i[0], node_i[1], node_i[2])  # real world coord
        return robot_traj

    def robot_controller(self, node_i, point_s):
        # This controller determines the velocities that will nominally move the robot from node i to node s
        # Max velocities should be enforced
        # alpha is the tuning parameter
        theta_d = np.arctan2((point_s[1] - node_i[1]), (point_s[0] - node_i[0]))
        theta = node_i[2]
        heading_error = theta_d - theta

        rot_vel = -4 * self.alpha * np.tan(heading_error)

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
            # print("TO DO: Implement a control scheme to drive you towards the sampled point")

        return vel, rot_vel

    def trajectory_rollout(self, vel=0, rot_vel=0, x0=0, y0=0, theta0=0):
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
            x = (vel / rot_vel) * (np.sin(rot_vel * t + theta0) - np.sin(theta0)) + x0
            y = (vel / rot_vel) * (np.cos(rot_vel * t + theta0) - np.cos(theta0)) + y0
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

    def cell_to_point(self, point):
        new_point = point.copy()
        new_point[0] = (point[0] * self.map_settings_dict["resolution"] + self.bounds[0, 0])
        new_point[1] = (point[1] * self.map_settings_dict["resolution"] + self.bounds[1, 0])

        return new_point

    def points_to_robot_circle(self, points):
        # Convert a series of [x,y] points to robot map footprints for collision detection
        # Hint: The disk function is included to help you with this function

        # print("TO DO: Implement a method to get the pixel locations of the robot path")

        rr = []
        cc = []
        robot_radius = np.floor(self.robot_radius / self.map_settings_dict["resolution"]).astype(int)

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
        print("TO DO: Implement a way to connect two already existing nodes (for rewiring).")
        return np.zeros((3, self.num_substeps))

    def cost_to_come(self, trajectory_o):
        # The cost to get to a node from lavalle

        # print("TO DO: Implement a cost to come metric")

        cost = 0.0
        for i in range(1, trajectory_o[0].shape()):
            x_d = trajectory_o[0, i] - trajectory_o[0, i - 1]
            y_d = trajectory_o[1, i] - trajectory_o[1, i - 1]

            cost += np.sqrt(x_d ** 2 + y_d ** 2)

        return cost

    def update_children(self, node_id):
        # Given a node_id with a changed cost, update all connected nodes with the new cost
        print("TO DO: Update the costs of connected nodes after rewiring.")
        return

    # Planner Functions
    def rrt_planning(self):
        # This function performs RRT on the given map and robot
        # You do not need to demonstrate this function to the TAs, but it is left in for you to check your work
        for i in range(50000):  # Most likely need more iterations than this to complete the map!
            # Sample map space
            point = self.sample_map_space()

            # Get the closest point
            closest_node_id = self.closest_node(point, 1)
            print("closet node", self.nodes[closest_node_id].point.reshape(3, ), "picked point", point.reshape(2, ))

            # Simulate driving the robot towards the closest point
            trajectory_o = self.simulate_trajectory(self.nodes[closest_node_id].point, point)

            # Check for collisions
            # print("TO DO: Check for collisions and add safe points to list of nodes.")
            collision = self.check_colision(trajectory_o)
            duplicate = self.check_if_duplicate(trajectory_o[:, -1])
            print("dup", duplicate)

            # collision=False
            if not (collision or duplicate):
                # Add node to list
                cost = 0
                self.nodes[closest_node_id].children_ids.append(self.nodes[-1].id + 1)
                self.nodes.append(
                    Node(np.array(trajectory_o[:, -1].reshape((3, 1))), closest_node_id, cost, self.nodes[-1].id + 1))

                # visualize
                print("added node", self.nodes[-1].id, self.nodes[-1].parent_id)
                self.window.add_se2_pose(np.array(trajectory_o[:, -1].reshape((3,))))
                self.vis(np.transpose(trajectory_o[0:2, :]))

            # Check if goal has been reached
            if np.linalg.norm(trajectory_o[0:2, -1] - self.goal_point.reshape(2, )) < self.stopping_dist:
                print("rrt success")
                return self.recover_path()
        print("rrt failed")
        return None

    def rrt_star_planning(self):
        # This function performs RRT* for the given map and robot
        for i in range(50000):  # Most likely need more iterations than this to complete the map!
            # Sample
            point = self.sample_map_space()

            # Closest Node
            closest_node_id = self.closest_node(point, 1)

            # Simulate trajectory
            trajectory_o = self.simulate_trajectory(self.nodes[closest_node_id].point, point)

            # Check for Collision
            collision = self.check_colision(trajectory_o)
            duplicate = self.check_if_duplicate(trajectory_o[:, -1])
            print("dup", duplicate)

            # collision=False
            if not (collision or duplicate):
                # Calculate cost
                if i > 0:
                    cost = self.cost_to_come(trajectory_o) + self.nodes[closest_node_id].cost
                else:
                    cost = 0

                pt = np.array(trajectory_o[:, -1].reshape((3, 1)))
                # self.connect_node_to_point(self.nodes[closest_node_id], pt)

                # add node
                self.nodes[closest_node_id].children_ids.append(self.nodes[-1].id + 1)
                self.nodes.append(Node(pt, closest_node_id, cost, self.nodes[-1].id + 1))

                if i > 6:
                    closest_nodes = self.closest_node(pt, 6)
                    cost_min = cost
                    cost_min_id = closest_node_id

                    for node in closest_nodes[1:]:  # ignore closest node
                        # Simulate trajectory from near nodes to pt
                        trajectory_node = self.connect_node_to_point(self.nodes[node].point, pt)

                        # Check for Collision
                        collision = self.check_colision(trajectory_node)
                        duplicate = self.check_if_duplicate(trajectory_node[:, -1])
                        if not (collision or duplicate):
                            # Calculate cost
                            if i > 0:
                                cost = self.cost_to_come(trajectory_node) + self.nodes[node].cost
                            else:
                                cost = 0

                            if cost < cost_min:  # get min cost
                                cost_min = cost
                                cost_min_id = node

                    if cost_min_id != closest_node_id:
                        # remove parent connection
                        self.nodes[closest_node_id].children_ids.remove(self.nodes[-1].id)

                        # add node to min cost node
                        self.nodes[cost_min_id].children_ids.append(self.nodes[-1].id)
                        self.nodes[-1].cost = cost_min
                        self.nodes[-1].parent_id = cost_min_id

                        # Rewire other nodes to pt
                        for node in closest_nodes:
                            # Skip if parent node
                            if node.id == cost_min_id:
                                continue

                            # Simulate trajectory from pt to near nodes
                            trajectory_node = self.connect_node_to_point(self.nodes[-1].point, self.nodes[node].point)

                            # Check for Collision
                            collision = self.check_colision(trajectory_node)
                            duplicate = self.check_if_duplicate(trajectory_node[:, -1])
                            if not (collision or duplicate):
                                # Calculate cost
                                if i > 0:
                                    cost = self.cost_to_come(trajectory_node) + self.nodes[-1].cost
                                else:
                                    cost = 0

                                # New path has less cost. Rewire
                                if cost < self.nodes[node].cost:
                                    # remove parent connection
                                    self.nodes[self.nodes[node].parent_id].children_ids.remove(node)

                                    # add node to pt
                                    self.nodes[node].cost = cost
                                    self.nodes[node].parent_id = self.nodes[-1].id

                                    # updating children
                                    self.nodes[self.nodes[-1].id].children_ids.append(node)
                                    self.update_children(node)

                # visualize
                print("added node", self.nodes[-1].id, self.nodes[-1].parent_id)
                self.window.add_se2_pose(np.array(trajectory_o[:, -1].reshape((3,))))
                self.vis(np.transpose(trajectory_o[0:2, :]))

            self.window.add_point(point.reshape(2, ))

            # Check if goal has been reached
            if np.linalg.norm(trajectory_o[0:2, -1] - self.goal_point.reshape(2, )) < self.stopping_dist:
                print("rrt success")
                return self.recover_path()

            print("rrt failed")

            # Last node rewire
            print("TO DO: Last node rewiring")

            # Close node rewire
            print("TO DO: Near point rewiring")

            # Check for early end
            print("TO DO: Check for early end")

        return self.nodes

    def recover_path(self, node_id=-1):
        path = [self.nodes[node_id].point]
        current_node_id = self.nodes[node_id].parent_id
        while current_node_id > -1:
            path.append(self.nodes[current_node_id].point)
            current_node_id = self.nodes[current_node_id].parent_id
        path.reverse()
        return path

    def vis(self, points):
        for i in points:
            self.window.add_point(i)
            # pass
        # self.window.display()


def main():
    # Set map information
    map_filename = "willowgarageworld_05res.png"
    # map_filename = "simple_map.png"
    map_setings_filename = "willowgarageworld_05res.yaml"

    # robot information
    goal_point = np.array([[10], [10]])  # m
    stopping_dist = 0.5  # m

    # RRT precursor
    path_planner = PathPlanner(map_filename, map_setings_filename, goal_point, stopping_dist)

    # nodes = path_planner.rrt_star_planning()
    # node_path_metric = np.hstack(path_planner.recover_path())
    #
    # # Leftover test functions
    # np.save("shortest_path.npy", node_path_metric)
    nodes = path_planner.rrt_planning()
    print(nodes)
    path_planner.window.display()


if __name__ == '__main__':
    main()
