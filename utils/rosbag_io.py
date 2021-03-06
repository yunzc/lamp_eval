import sys
import os
import ctypes
import numpy
import rosbag
import rospy

import geometry_utils as gu
from os.path import exists

def read_odom_bag(bag_name, odom_topic):
    poses = []
    if not exists(bag_name):
        print("Cannot find {}. ".format(bag_name))
        return poses

    with rosbag.Bag(bag_name) as bag:
        for topic, message, t in bag.read_messages(topics=[odom_topic]):
            new_pose = gu.PoseStamped(message.header.stamp.to_nsec())
            new_pose.pose.from_odom_msg(message)
            poses.append(new_pose)
    return poses

def read_traj_from_pg_bag(bag_name, pose_graph_topic, incremental=True):
    poses = []
    keys = []
    last_msg = None
    if not exists(bag_name):
        print("Cannot find {}. ".format(bag_name))
        return poses

    with rosbag.Bag(bag_name) as bag:
        for topic, message, t in bag.read_messages(topics=[pose_graph_topic]):
            if incremental:
                for node in message.nodes:
                    if node.key not in keys:
                        new_pose = gu.PoseKeyedStamped(node.header.stamp.to_nsec(), node.key)
                        new_pose.pose.from_pose_msg(node.pose)
                        keys.append(node.key)
                        poses.append(new_pose)
            else:
                last_msg = message
    if not incremental:
        for node in last_msg.nodes:
            if node.key not in keys:
                new_pose = gt.PoseKeyedStamped(node.header.stamp.to_nsec(), node.key)
                new_pose.pose.from_pose_msg(node.pose)
                keys.append(node.key)
                poses.append(new_pose)
    return poses

def read_lc_from_pg_bag(bag_name, pose_graph_topic):
    key_from = []
    key_to = []
    poses = []
    if not exists(bag_name):
        print("Cannot find {}. ".format(bag_name))
        return key_from, key_to, poses

    with rosbag.Bag(bag_name) as bag:
        for topic, message, t in bag.read_messages(topics=[pose_graph_topic]):
            for edge in message.edges:
                if edge.type == 3:
                    key_from.append(edge.key_from)
                    key_to.append(edge.key_to)
                    edge_pose = Pose()
                    edge_pose.from_pose_msg(edge.pose)
                    poses.append(edges_pose)
    return key_from, key_to, poses