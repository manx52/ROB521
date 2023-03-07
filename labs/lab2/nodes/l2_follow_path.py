#!/usr/bin/env python3
from __future__ import division, print_function

import math
import os

import numpy as np
from scipy.linalg import block_diag
from scipy.spatial.distance import cityblock
import rospy
import tf2_ros

# msgs
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from skimage.transform._hough_transform import circle_perimeter
from visualization_msgs.msg import Marker

# ros and se2 conversion utils
import utils

TRANS_GOAL_TOL = .2  # m, tolerance to consider a goal complete
ROT_GOAL_TOL = .3  # rad, tolerance to consider a goal complete
TRANS_VEL_OPTS = [0.00, 0.025, 0.13]  # m/s, max of real robot is .26
# ROT_VEL_OPTS = np.linspace(-1.82, 1.82, 11)  # rad/s, max of real robot is 1.82
# TRANS_VEL_OPTS = [0, 0.025, 0.13, 0.1]  # m/s, max of real robot is .26
ROT_VEL_OPTS = np.linspace(-0.51, 0.51, 11)  # rad/s, max of real robot is 1.82
CONTROL_RATE = 5  # Hz, how frequently control signals are sent
CONTROL_HORIZON = 5  # seconds. if this is set too high and INTEGRATION_DT is too low, code will take a long time to run!
INTEGRATION_DT = .025  # s, delta t to propagate trajectories forward by
COLLISION_RADIUS = 0.225  # m, radius from base_link to use for collisions, min of 0.2077 based on dimensions of .281 x .306
ROT_DIST_MULT = .1  # multiplier to change effect of rotational distance in choosing correct control
OBS_DIST_MULT = .1  # multiplier to change the effect of low distance to obstacles on a path
MIN_TRANS_DIST_TO_USE_ROT = TRANS_GOAL_TOL  # m, robot has to be within this distance to use rot distance in cost
PATH_NAME = 'path.npy'  # saved path from l2_planning.py, should be in the same directory as this file

# here are some hardcoded paths to use if you want to develop l2_planning and this file in parallel
# TEMP_HARDCODE_PATH = [[2, 0, 0], [2.75, -1, -np.pi/2], [2.75, -4, -np.pi/2], [2, -4.4, np.pi]]  # almost collision-free
TEMP_HARDCODE_PATH = [[2, -.5, 0], [2.4, -1, -np.pi / 2], [2.45, -3.5, -np.pi / 2],
                      [1.5, -4.4, np.pi]]  # some possible collisions

import scipy.spatial as sp
def load_map(filename):
    import matplotlib.image as mpimg
    import cv2
    print(os.getcwd())
    im = mpimg.imread("../maps/" + filename)
    if len(im.shape) > 2:
        im = im[:, :, 0]
    im_np = np.array(im)  # Whitespace is true, black is false
    # im_np = np.logical_not(im_np)     #for ros
    return im_np


class PathFollower():
    def __init__(self):

        # time full path
        self.path_follow_start_time = rospy.Time.now()

        # use tf2 buffer to access transforms between existing frames in tf tree
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.0)  # time to get buffer running

        # constant transforms
        self.map_odom_tf = self.tf_buffer.lookup_transform('map', 'odom', rospy.Time(0), rospy.Duration(2.0)).transform
        print(self.map_odom_tf)

        # subscribers and publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.global_path_pub = rospy.Publisher('~global_path', Path, queue_size=1, latch=True)
        self.local_path_pub = rospy.Publisher('~local_path', Path, queue_size=1)
        self.collision_marker_pub = rospy.Publisher('~collision_marker', Marker, queue_size=1)

        # map
        map = rospy.wait_for_message('/map', OccupancyGrid)
        self.map_np = np.array(map.data).reshape(map.info.height, map.info.width)
        self.map_resolution = round(map.info.resolution, 5)
        self.map_origin = -utils.se2_pose_from_pose(map.info.origin)  # negative because of weird way origin is stored
        print(self.map_origin)
        self.map_nonzero_idxes = np.argwhere(self.map_np)
        # map_filename = "willowgarageworld_05res.png"
        # occupancy_map = load_map(map_filename)
        # self.map_np = occupancy_map
        # self.map_resolution = 0.050000
        # # self.map_origin = np.array([-21.0, -49.25, 0.000000])
        # self.map_origin = np.array([19.0, -9.25, 0.000000])
        # self.map_nonzero_idxes = np.argwhere(self.map_np)
        # print(map)

        self.bounds = np.zeros([2, 2])  # m
        self.bounds[0, 0] = self.map_origin[0]
        self.bounds[1, 0] = self.map_origin[1]
        self.bounds[0, 1] = self.map_origin[0] + self.map_np.shape[1] * self.map_resolution
        self.bounds[1, 1] = self.map_origin[1] + self.map_np.shape[0] * self.map_resolution

        # collisions
        self.collision_radius_pix = COLLISION_RADIUS / self.map_resolution
        self.collision_marker = Marker()
        self.collision_marker.header.frame_id = '/map'
        self.collision_marker.ns = '/collision_radius'
        self.collision_marker.id = 0
        self.collision_marker.type = Marker.CYLINDER
        self.collision_marker.action = Marker.ADD
        self.collision_marker.scale.x = COLLISION_RADIUS * 2
        self.collision_marker.scale.y = COLLISION_RADIUS * 2
        self.collision_marker.scale.z = 1.0
        self.collision_marker.color.g = 1.0
        self.collision_marker.color.a = 0.5

        # transforms
        self.map_baselink_tf = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        self.pose_in_map_np = np.zeros(3)
        self.pos_in_map_pix = np.zeros(2)
        self.update_pose()

        # path variables
        cur_dir = os.path.dirname(os.path.realpath(__file__))

        # to use the temp hardcoded paths above, switch the comment on the following two lines
        self.path_tuples = np.load(os.path.join(cur_dir, 'shortest_path_rrtstar.npy')).T
        # self.path_tuples = np.load(os.path.join(cur_dir, 'path_complete.npy')).T
        # self.path_tuples = np.array(TEMP_HARDCODE_PATH)

        self.path = utils.se2_pose_list_to_path(self.path_tuples, 'map')
        self.global_path_pub.publish(self.path)

        # goal
        self.cur_goal = np.array(self.path_tuples[0])
        self.cur_path_index = 0

        # trajectory rollout tools
        # self.all_opts is a Nx2 array with all N possible combinations of the t and v vels, scaled by integration dt
        self.all_opts = np.array(np.meshgrid(TRANS_VEL_OPTS, ROT_VEL_OPTS)).T.reshape(-1, 2)

        # if there is a [0, 0] option, remove it
        all_zeros_index = (np.abs(self.all_opts) < [0.001, 0.001]).all(axis=1).nonzero()[0]
        if all_zeros_index.size > 0:
            self.all_opts = np.delete(self.all_opts, all_zeros_index, axis=0)
        self.all_opts_scaled = self.all_opts * INTEGRATION_DT

        self.num_opts = self.all_opts_scaled.shape[0]
        self.horizon_timesteps = int(np.ceil(CONTROL_HORIZON / INTEGRATION_DT))

        self.rate = rospy.Rate(CONTROL_RATE)

        rospy.on_shutdown(self.stop_robot_on_shutdown)
        self.follow_path()

    def follow_path(self):
        while not rospy.is_shutdown():
            # timing for debugging...loop time should be less than 1/CONTROL_RATE
            tic = rospy.Time.now()

            self.update_pose()
            self.check_and_update_goal()

            # start trajectory rollout algorithm
            local_paths = np.zeros([self.horizon_timesteps + 1, self.num_opts, 3])
            local_paths[0] = np.atleast_2d(self.pose_in_map_np).repeat(self.num_opts, axis=0)
            collision_traj_idx = []
            # print("TO DO: Propogate the trajectory forward, storing the resulting points in local_paths!")
            for t in range(0, self.num_opts):
                #     propogate trajectory forward, assuming perfect control of velocity and no dynamic effects
                local_paths[:, t, :] = self.trajectory_rollout2(self.all_opts[t, 0], self.all_opts[t, 1],
                                                                self.pose_in_map_np).T
                # pass
                # local_paths[:, t, :] = self.trajectory_rollout(self.all_opts[t, 0], self.all_opts[t, 1],
                #                                                self.pose_in_map_np[0], self.pose_in_map_np[1],
                #                                                self.pose_in_map_np[2]).T
                # local_paths[:, t, :] = self.trajectory_rollout3(self.all_opts[t, 0], self.all_opts[t, 1]).T
                # if not self.check_colision(local_paths[:, t, :]):
                #     collision_traj_idx.append(t)

            #
            # # check all trajectory points for collisions
            # # first find the closest collision point in the map to each local path point
            # local_paths_pixels = (-self.map_origin[:2]+local_paths[:, :, :2]) / self.map_resolution
            # #print(local_paths_pixels.shape)
            # local_paths_pixels[:,:,1]=1600-local_paths_pixels[:,:,1]
            # #print(self.map_np.shape)
            # valid_opts = range(self.num_opts)
            # local_paths_lowest_collision_dist = np.ones(self.num_opts) * 50
            # # print("Pixel: ", local_paths_pixels[0,0,:])
            # # print("real: ", local_paths[0, 0, :])
            # collision_traj_idx = []
            # for opt in range(local_paths_pixels.shape[1]):
            #     x = local_paths_pixels[:, opt, 1]
            #     y = local_paths_pixels[:, opt, 0]
            #     # x = np.clip(x, 0, self.map_np.shape[1] - 1)
            #     # y = np.clip(y, 0, self.map_np.shape[1] - 1)
            #
            #     for p in range(len(x)):
            #         if self.map_np[int(x[p]), int(y[p])] == 0:
            #             collision_traj_idx.append(opt)
            #             break
            local_paths_pixels = (self.map_origin[:2] + local_paths[:, :, :2]) / self.map_resolution
            valid_opts = range(self.num_opts)
            print("Pixel: ", local_paths_pixels[0,0,1],local_paths_pixels[0,0,0])
            print("real: ", local_paths[0, 0, :])
            local_paths_lowest_collision_dist = np.ones(self.num_opts) * 20
            close_wall = [i for i in self.map_nonzero_idxes if np.linalg.norm(i - [self.pos_in_map_pix[1],self.pos_in_map_pix[0]]) < 20]
            if len(close_wall) > 0:
                for opt in range(local_paths_pixels.shape[1]):
                    for timestep in range(local_paths_pixels.shape[0]):
                        curr_loc = np.array([int(local_paths_pixels[timestep,opt,1]),int(local_paths_pixels[timestep,opt,0])])
                        kdtree = sp.cKDTree(close_wall)
                        d, i = kdtree.query(curr_loc.T, k=1)
                        # print(d)
                        if d < self.collision_radius_pix:
                            collision_traj_idx.append(opt)
                            break

            valid_opts = np.delete(np.array(valid_opts), collision_traj_idx)

            # calculate final cost and choose best option
            # print("TO DO: Calculate the final cost and choose the best control option!")

            final_cost = np.zeros(self.num_opts)
            for i in range(self.num_opts):
                if i in valid_opts:
                    # final_cost[i] = self.cost_to_go(local_paths[:, i, :].T)
                    final_cost[i] = self.cost_to_come(local_paths[:, i, :].T)
                    # print("*****************in first cond")
                else:
                    final_cost[i] = np.Inf
                    # print("in else")
            # print("cost to go", final_cost)
            print(final_cost)
            # print(local_paths)
            if final_cost.min == np.Inf:  # hardcoded recovery if all options have collision
                control = [-.1, 0]
            else:
                best_opt = final_cost.argmin()
                control = self.all_opts[best_opt]
                self.local_path_pub.publish(utils.se2_pose_list_to_path(local_paths[:, best_opt], 'map'))

            # send command to robot
            self.cmd_pub.publish(utils.unicyle_vel_to_twist(control))

            # uncomment out for debugging if necessary
            # print("Selected control: {control}, Loop time: {time}, Max time: {max_time}".format(
            #     control=control, time=(rospy.Time.now() - tic).to_sec(), max_time=1/CONTROL_RATE))

            self.rate.sleep()

    def check_colision(self, robot_traj):
        # pass in trajectory = 3 X N from simulate_trajectory() and check for collision
        # False -> collision detected
        # True -> no collision
        traj_rr, traj_cc = self.points_to_robot_circle(
            robot_traj[0:2, :])  # center and radius of trajectory in occupacy map
        footprint = np.moveaxis(np.array([traj_rr, traj_cc]), 0, 2)
        temp_x = np.clip(footprint[..., 1], 0, self.map_np.shape[0] - 1)
        temp_y = np.clip(footprint[..., 0], 0, self.map_np.shape[1] - 1)

        return np.any(np.any(self.map_np[temp_x, temp_y] == 0, axis=-1))
        # for p in range(len):
        #     if int(self.map_np[int(x[p]), int(y[p])]) == 0:
        #             collision_traj_idx.append(opt)
        #             break

    def points_to_robot_circle(self, points):
        # Convert a series of [x,y] points to robot map footprints for collision detection
        # Hint: The disk function is included to help you with this function

        # print("TO DO: Implement a method to get the pixel locations of the robot path")

        rr = []
        cc = []

        for pt in (np.transpose(points)):
            # occ = self.point_to_cell(pt)

            temp_rr, temp_cc = circle_perimeter(int(pt[0]), int(pt[1]), int(self.collision_radius_pix))

            rr.append(temp_rr)
            cc.append(temp_cc)
        return np.array(rr), np.array(cc)

    def point_to_cell(self, point):
        # Convert a series of [x,y] points in the map to the indices for the corresponding cell in the occupancy map
        # point is a 2 by N matrix of points of interest

        # print("TO DO: Implement a method to get the map cell the robot is currently occupying")

        new_point = point.copy()
        # new_point[0] = (-self.map_origin[0] + point[0]) / self.map_resolution
        # new_point[1] = (-self.map_origin[1] + point[1]) / self.map_resolution
        new_point[0] = (point[0] - self.map_origin[0]) / self.map_resolution
        new_point[1] = self.map_origin[0] - (point[1] - self.map_origin[1]) /self.map_resolution
        new_point = (np.floor(new_point)).astype(int)
        return new_point

    def cost_to_come(self, trajectory_o):
        # The cost to get to a node from lavalle

        # print("TO DO: Implement a cost to come metric")

        # cost = 0.0
        # # Path distance
        # for i in range(1, len(trajectory_o[0])):
        #     x_d = trajectory_o[0, i] - trajectory_o[0, i - 1]
        #     y_d = trajectory_o[1, i] - trajectory_o[1, i - 1]
        #
        #     cost += np.sqrt(x_d ** 2 + y_d ** 2)
        #
        # return cost
        # x_d = self.cur_goal[0] - self.pose_in_map_np[0]
        # y_d = self.cur_goal[1] - self.pose_in_map_np[1]
        # theta_s = math.atan2(y_d, x_d)
        # heading_error = theta_s - self.pose_in_map_np[2]
        # heading_error_norm = math.atan2(math.sin(heading_error), math.cos(heading_error))
        #
        # lateral_error = math.sqrt(x_d**2 + y_d**2)
        #
        # cost = heading_error_norm + lateral_error
        end_pt = trajectory_o[:, -1]
        cost = np.linalg.norm(self.cur_goal[:2] - end_pt[:2])

        return cost

    def trajectory_rollout2(self, vel, rot_vel, node):
        # Given your chosen velocities determine the trajectory of the robot for your given timestep
        # The returned trajectory should be a series of points to check for collisions
        robot_pose = node
        traj = np.zeros((3, self.horizon_timesteps + 1))
        real_bounds = np.array([[0.0, 43.5], [-45, 10]])
        for i in range(self.horizon_timesteps + 1):
            traj[0, i] = robot_pose[0] + vel * INTEGRATION_DT * math.cos(robot_pose[2])
            traj[1, i] = robot_pose[1] + vel * INTEGRATION_DT * math.sin(robot_pose[2])
            traj[2, i] = robot_pose[2] + rot_vel * INTEGRATION_DT

            robot_pose = traj[:, i]
        return traj

    def trajectory_rollout(self, vel=0, rot_vel=0, x0=0, y0=0, theta0=0):
        # Given your chosen velocities determine the trajectory of the robot for your given timestep
        # The returned trajectory should be a series of points to check for collisions
        # re_turn 3 x n
        t = np.linspace(0, INTEGRATION_DT, self.horizon_timesteps + 1)
        x0 = np.ones((1, self.horizon_timesteps + 1)) * x0
        y0 = np.ones((1, self.horizon_timesteps + 1)) * y0
        theta0 = np.ones((1, self.horizon_timesteps + 1)) * theta0
        if rot_vel == 0:
            x = vel * t * np.cos(theta0) + x0
            y = vel * t * np.sin(theta0) + y0
            theta = np.zeros_like(t)
        else:
            x = (vel / rot_vel) * (np.sin(rot_vel * t + theta0) - np.sin(theta0)) + x0
            # offset = 0.5
            # x = np.clip(x, self.bounds[0, 0] + offset, self.bounds[0, 1] - offset)
            y = -(vel / rot_vel) * (np.cos(rot_vel * t + theta0) - np.cos(theta0)) + y0
            # y = np.clip(y, self.bounds[1, 0] + offset, self.bounds[1, 1] - offset)
            theta = (rot_vel * t + theta0) % (2 * math.pi)
        est_trajectory = np.vstack((x, y, theta))

        return est_trajectory

    def update_pose(self):
        # Update numpy poses with current pose using the tf_buffer
        self.map_baselink_tf = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0)).transform
        self.pose_in_map_np[:] = [self.map_baselink_tf.translation.x, self.map_baselink_tf.translation.y,
                                  utils.euler_from_ros_quat(self.map_baselink_tf.rotation)[2]]
        self.pos_in_map_pix = (self.map_origin[:2] + self.pose_in_map_np[:2]) / self.map_resolution
        self.collision_marker.header.stamp = rospy.Time.now()
        self.collision_marker.pose = utils.pose_from_se2_pose(self.pose_in_map_np)
        self.collision_marker_pub.publish(self.collision_marker)

    def check_and_update_goal(self):
        # iterate the goal if necessary
        dist_from_goal = np.linalg.norm(self.pose_in_map_np[:2] - self.cur_goal[:2])
        abs_angle_diff = np.abs(self.pose_in_map_np[2] - self.cur_goal[2])
        rot_dist_from_goal = min(np.pi * 2 - abs_angle_diff, abs_angle_diff)
        if dist_from_goal < TRANS_GOAL_TOL:  # and rot_dist_from_goal < ROT_GOAL_TOL:
            rospy.loginfo("Goal {goal} at {pose} complete.".format(
                goal=self.cur_path_index, pose=self.cur_goal))
            if self.cur_path_index == len(self.path_tuples) - 1:
                rospy.loginfo("Full path complete in {time}s! Path Follower node shutting down.".format(
                    time=(rospy.Time.now() - self.path_follow_start_time).to_sec()))
                rospy.signal_shutdown("Full path complete! Path Follower node shutting down.")
            else:
                self.cur_path_index += 1
                self.cur_goal = np.array(self.path_tuples[self.cur_path_index])
        else:
            rospy.logdebug("Goal {goal} at {pose}, trans error: {t_err}, rot error: {r_err}.".format(
                goal=self.cur_path_index, pose=self.cur_goal, t_err=dist_from_goal, r_err=rot_dist_from_goal
            ))

    def stop_robot_on_shutdown(self):
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Published zero vel on shutdown.")

    def trajectory_rollout3(self, vel, rot_vel):
        # Given your chosen velocities determine the trajectory of the robot for your given timestep
        # The returned trajectory should be a series of points to check for collisions

        theta = np.zeros(self.horizon_timesteps + 1)
        x = np.zeros(self.horizon_timesteps + 1)
        y = np.zeros(self.horizon_timesteps + 1)
        for i in range(1, self.horizon_timesteps + 1):
            theta[i] = rot_vel * i * INTEGRATION_DT

            x[i] = ((1.0 * vel) / rot_vel) * np.sin(theta[i])
            y[i] = ((1.0 * vel) / rot_vel) * (1 - np.cos(theta[i]))

        robot_traj = np.array((x, y, theta))

        robot_theta = self.pose_in_map_np[2]

        for i in range(self.horizon_timesteps + 1):
            x_old = robot_traj[0, i]
            y_old = robot_traj[1, i]
            robot_traj[0, i] = x_old * np.cos(robot_theta) - y_old * np.sin(robot_theta) + self.pose_in_map_np[0]
            robot_traj[1, i] = x_old * np.sin(robot_theta) + y_old * np.cos(robot_theta) + self.pose_in_map_np[1]
            robot_traj[2, i] += robot_theta % (2 * np.pi)

        return robot_traj


if __name__ == '__main__':
    try:
        rospy.init_node('path_follower', log_level=rospy.DEBUG)
        pf = PathFollower()
    except rospy.ROSInterruptException:
        pass
