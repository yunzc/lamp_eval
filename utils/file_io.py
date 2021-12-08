import geometry_utils as gu
import csv

def read_traj_from_g2o(filename):
	f = open(filename, "r")
	poses = []
	for line in f:
		if line.startswith("VERTEX_SE3:QUAT"):
			# VERTEX_SE3:QUAT 7133701809754865664 -7.81016 215.785 1.94888 0.005279 -0.020584 -0.004705 0.999763
			line_split = line.split()
			key = int(line_split[1])
			pose = Pose()
			pose.set_translation(np.array([line_split[2], line_split[3], line_split[4]]))
			pose.set_rotation(np.array([line_split[5], line_split[6], line_split[7], line_split[8]]))
			poses.append(PoseKeyed(key, pose))
	return poses

def write_traj_to_g2o(poses, filename):
	# poses is a list of type PoseKeyed
	with open(filename, 'w') as f:
		for pose in poses:
			f.write("VERTEX_SE3:QUAT " + str(pose.key) + " " + str(pose.pose) + "\n")
	print("Wrote trajectory with {} poses to {}".format(str(len(poses)), filename))