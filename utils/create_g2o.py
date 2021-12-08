import os
import argparse
from rosbag_io import read_odom_bag
from rosbag_io import read_traj_from_pg_bag
from file_io import write_traj_to_g2o
import numpy as np

def list_all_bags(folder):
    rosbags = []
    for file in os.listdir(folder):
        if file.endswith("bag"):
            rosbags.append(folder + "/" + file)
    return rosbags

def generate_gt_traj(poses_keyed_stamped, gt_poses_stamped):
    gt_poses = []
    for p in poses_keyed_stamped:
        closest_ps = None
        t_diff = np.inf
        for gt_p in gt_poses_stamped:
            if t_diff > abs(gt_p.stamp - p.stamp):
                t_diff = abs(gt_p.stamp - p.stamp)
                closest_ps = gt_p
        if closest_ps is None or gt_p > 1e5:
            print("Unable to find pose with corresponding stamp in gt poses. ")
            continue
        gt_pks = p
        gt_pks.pose = closest_ps.pose
        gt_poses.append(gt_pks)
    return gt_poses

def main():
    parser = argparse.ArgumentParser(description="Create g2o file of trajectory from pose graph and odom rosbags. ")
    parser.add_argument("path", type=str, help="path to folder (with rosbag file and ground_truth file).")
    parser.add_argument("robots", type=str, help="robot names separated by comma and no space. ")
    args = parser.parse_args()

    gt_odom_bags = list_all_bags(args.path + "/ground_truth")
    pg_bags = list_all_bags(args.path + "/rosbag")

    gt_traj = []
    for robot_name in args.robots.split(","):
        gt_bag = None
        pg_bag = None
        for bag in gt_odom_bags:
            if bag.split("/")[-1].startswith(robot_name):
                gt_bag = bag
        if gt_bag is None:
            continue
        for bag in pg_bags:
            if bag.split("/")[-1].startswith(robot_name):
                pg_bag = bag
        if pg_bag is None:
            continue

        # read bags and generate gt traj
        robot_gt_odom = read_odom_bag(gt_bag, "/{}/lo_frontend/odometry".format(robot_name))
        robot_pg = read_traj_from_pg_bag(pg_bag, "/{}/lamp/pose_graph_incremental".format(robot_name))

        robot_gt_traj = generate_gt_traj(robot_pg, robot_gt_odom)
        if len(robot_gt_traj) == 0:
            print("WARNING: generated gt pose graph has length 0. ")
        gt_traj.extend(robot_gt_traj)

    output_g2o = args.path + "/ground_truth/result.g2o"
    write_traj_to_g2o(gt_traj, output_g2o)

if __name__ == '__main__':
    main()

