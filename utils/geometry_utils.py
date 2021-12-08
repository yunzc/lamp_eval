import numpy as np
from scipy.spatial.transform import Rotation as Rot
from nav_msgs.msg import Odometry

def to_pose_matrix(t, q):
	# t = [x, y, z]
	# q = [x, y, z, w]
	R = Rot.from_quat(q)
	T = np.eye(4)
	T[0:3, 0:3] = R.as_dcm()
	T[0:3, 3] = t
	return T

def from_pose_matrix(T):
	R = T[0:3, 0:3]
	t = T[0:3, 3]
	R = Rot.from_dcm(R)
	q = R.as_quat()
	return t, q


class Pose:
	def __init__(self):
		self.rot = Rot.from_quat(np.array([0, 0, 0,1]))
		self.t = np.array([0, 0, 0])

	def matrix(self):
		T = np.eye(4)
		T[0:3, 0:3] = self.rot.as_dcm()
		T[0:3, 3] = self.t

	def set_translation(self, t):
		self.t = t

	def set_rotation(self, quat):
		self.rot = Rot.from_quat(quat)

	def quat(self):
		return self.rot.as_quat()

	def trans(self):
		return self.t

	def from_odom_msg(self, odom_msg):
		self.from_pose_msg(odom_msg.pose.pose)

	def from_pose_msg(self, pose_msg):
		self.t = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
		self.rot = Rot.from_quat(np.array([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]))

	def __str__(self):
		t = self.t
		q = self.quat()
		return "{} {} {} {} {} {} {}".format(str(t[0]), str(t[1]), str(t[2]), str(q[0]), str(q[1]), str(q[2]), str(q[3]))

class PoseStamped:
	def __init__(self, stamp):
		self.stamp = stamp
		self.pose = Pose()

class PoseKeyed:
	def __init___(self, key):
		self.key = key
		self.pose = Pose()

class PoseKeyedStamped:
	def __init__(self, stamp, key):
		self.stamp = stamp
		self.key = key
		self.pose = Pose()

