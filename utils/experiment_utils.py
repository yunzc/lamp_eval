from file_io import read_traj_from_g2o
from rosbag_io import read_lc_from_pg_bag

import key_handling as kh

def evaluate_g2o(input, ground_truth):
	poses = read_traj_from_g2o(input)
	gt_poses = read_traj_from_g2o(ground_truth)

	t_error = {}
	r_error = {}

	for pose in poses:
		gt_pose = next((i for i in gt_poses if i.key == pose.key), None)
		te, re = pose.pose.error(gt_pose.pose)
		prefix, index = kh.split_pg_key(pose.key)
		if prefix not in t_error:
			t_error[prefix] = []
			r_error[prefix] = []
		t_error[prefix].append(te)
		r_error[prefix].append(re)

	output_ate = {}
	output_are = {}
	for prefix in t_error:
		if prefix in kh.prefix_to_name:
			output_ate[kh.prefix_to_name[prefix]] = sum(t_error[prefix]) / len(t_error[prefix])
			output_are[kh.prefix_to_name[prefix]] = sum(r_error[prefix]) / len(r_error[prefix])
	return output_ate, output_are

def evaluate_lc(lc_bag, gt_g2o, lc_topic):
	key_from, key_to, transform = read_lc_from_pg_bag(lc_bag, lc_topic)
	gt_poses = read_traj_from_g2o(gt_g2o)

	lc_ate = []
	lc_are = []
	for i in range(len(key_from)):
		kf = key_from[i]
		kt = key_to[i]
		tf = transform[i]

		gt_pose_kf = next((i for i in gt_poses if i.key == kf), None)
		gt_pose_kt = next((i for i in gt_poses if i.key == kt), None)
		gt_tf = gt_pose_kt.pose - gt_pose_kf

		te, re = tf.error(gt_tf)

		lc_ate.append(te)
		lc_are.append(re)

	return lc_ate, lc_are


