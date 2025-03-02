#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np

import std_msgs.msg
import moveit_msgs.msg
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity

from ur5_inv_kin import ur5
from const import *

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
			'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
UR5_LINK = ['base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 
				'wrist_1_link', 'wrist_2_link', 'wrist_3_link']
ROBOTIQ_LINK = ['robotiq_arg2f_base_link', 'left_outer_knuckle', 'left_outer_finger', 
				'left_inner_finger', 'left_inner_finger_pad', 'left_inner_knuckle', 
				'right_inner_knuckle', 'right_outer_knuckle', 'right_outer_finger', 
				'right_inner_finger', 'right_inner_finger_pad']

LINKS_ENABLE = UR5_LINK + ROBOTIQ_LINK
LINKS_DISABLE = ['realsense2_link', 'part'] # 'part' doesn't work

rad2deg = 180.0/math.pi
deg2rad = math.pi/180.0

class ur5_inv_kin_wrapper(ur5):
	def __init__(self):
		ur5.__init__(self)

		# Publisher
		self.robot_state_pub = rospy.Publisher('/inv_kin_sol', moveit_msgs.msg.DisplayRobotState, queue_size=1)
		self.diff_pub = rospy.Publisher('/inv_kin_diff', std_msgs.msg.Float32MultiArray, queue_size=1)
		
		# Subscriber
		self.aco_sub = rospy.Subscriber('/attached_collision_object', moveit_msgs.msg.AttachedCollisionObject, self.aco_cb)

		# Service
		self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
		self.sv_srv.wait_for_service()
		print('service is available')
		
		# variables
		self.inv_sol = None
		self.w = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]	## edit

		self.aco = None

	def _convert_base_axis(self, pose_mat):
		# pose_mat : [4x4] array
		# /base_link is rotated 180 deg along z axis compared to real robot
		z_180 = tf.transformations.euler_matrix(0, 0, math.pi)
		pose_mat = tf.transformations.concatenate_matrices(z_180, pose_mat)

		return pose_mat

	def _tf_to_mat(self, trans, rot):
		# pose_mat : [4x4] array
		trans_mat = tf.transformations.translation_matrix(trans)
		rot_mat = tf.transformations.quaternion_matrix(rot)
		pose_mat = tf.transformations.concatenate_matrices(trans_mat, rot_mat)

		# pose_mat = self._convert_base_axis(pose_mat)
		return pose_mat

	def _print_sol(self, sol_rad):
		print('-'*70)
		print("IK solve")
		for i in range(8):
			sol_deg = sol_rad[:, i] * rad2deg
			print("sol {} : [{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(
				i, sol_deg[0], sol_deg[1], sol_deg[2], sol_deg[3], sol_deg[4], sol_deg[5]))

	def _calculate_diff(self, inv_sol, cur_joint):
		diffs = []
		inv_sol_diff = []
		for i in range(8):
			diff = 0.0
			for j in range(6): 
				diff += self.w[j] * (inv_sol[j][i] - cur_joint[j])**2 
			diffs.append(diff)
			inv_sol_diff.append({'joint': list(inv_sol[:, i]), 'diff': float(diff)})		
		print('diff: {}'.format(diffs))

		return inv_sol_diff

	def _solve(self, trans, rot):
		pose_mat = self._tf_to_mat(trans, rot)
		# inv_sol = self._inv_kin_limited(pose_mat)
		inv_sol = self.inv_kin(pose_mat)
		
		return inv_sol

	def _test(self, inv_sol, cur_joint):
		'''
		inv_sol : [-180, 180]
		'''
		for j in range(6):
			for i in range(8):
				cur = inv_sol[j, i]
				if cur < 0:
					tmp = cur + 2*math.pi
					if abs(tmp-cur_joint[j]) < abs(cur-cur_joint[j]):
						inv_sol[j, i] = tmp
				elif cur > 0:
					tmp = cur - 2*math.pi
					if abs(tmp-cur_joint[j]) < abs(cur-cur_joint[j]):
						inv_sol[j, i] = tmp
				else:
					tmp1 = cur + 2*math.pi
					tmp2 = cur - 2*math.pi
					if abs(tmp1-cur_joint[j]) < abs(cur-cur_joint[j]):
						if abs(tmp2-cur_joint[j]) < abs(tmp1-cur_joint[j]):
							inv_sol[j, i] = tmp2
						else:
							inv_sol[j, i] = tmp1

		return inv_sol

  	def aco_cb(self, msg):
  		self.aco = msg

	def _get_state_validity(self, joint):
		print(joint)
		robot_state = moveit_msgs.msg.RobotState()
		robot_state.joint_state.name = JOINTS
		robot_state.joint_state.position = joint
		
		if self.aco is not None:
			robot_state.attached_collision_objects = [self.aco]

		gsvr = GetStateValidityRequest()
		gsvr.robot_state = robot_state

		result = self.sv_srv.call(gsvr)
		rv = result.valid
		rc = result.contacts
		
		print("+"*70)
		print('state validity result')
		print('valid = {}'.format(rv))
		for i in range(len(rc)):
			print('contact {}: 1-{}, 2-{}'.format(i, rc[i].contact_body_1, rc[i].contact_body_2))

		return rv

	def _set_link_colors(self, r, g, b, a):
		link_colors = []

		for i in range(len(LINKS_ENABLE)):
			color = moveit_msgs.msg.ObjectColor()
			color.id = LINKS_ENABLE[i]
			color.color.r = r
			color.color.g = g
			color.color.b = b
			color.color.a = a
			link_colors.append(color)
		
		for i in range(len(LINKS_DISABLE)):
			color = moveit_msgs.msg.ObjectColor()
			color.id = LINKS_DISABLE[i]
			color.color.a = 0.0
			link_colors.append(color)

		return link_colors

	def solve_and_sort(self, trans, rot, cur_joint):
		'''
		inv_sol, inv_sol_sorted : list of list [6x8]
		inv_sol_diff : list of dict {'joint':, 'diff'}x8
		'''
		inv_sol = self._solve(trans, rot)
		inv_sol_near = self._test(inv_sol, cur_joint)
		inv_sol_diff = self._calculate_diff(inv_sol_near, cur_joint)
		# print(inv_sol_diff)

		def sortKey(e):
			return e['diff']
		
		inv_sol_diff.sort(key=sortKey)
		# print(inv_sol_diff)

		inv_sol_sorted = np.zeros((6, 8))
		for i in range(8):
			for j in range(6):
				inv_sol_sorted[j][i] = inv_sol_diff[i]['joint'][j]
		# print(inv_sol_sorted)

		print('current', cur_joint)
		self._print_sol(inv_sol_sorted)
		self.inv_sol = inv_sol_sorted

	def publish_state(self, num):
		'''
		publishes selected sol of robot state
		if num is -1, 
			publish transparent robot & return zeros
		else,
			1) check state validity
			2) if valid, publish green robot
					else, publish red robot
		'''
		print("="*80)
		print("publish state")
		
		valid = False

		if num == -1:
			selected_inv_sol = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
			link_colors = self._set_link_colors(0, 0, 0, 0)
		else:
			selected_inv_sol = self.inv_sol[:, num]
			valid = self._get_state_validity(selected_inv_sol)
			if valid:
				link_colors = self._set_link_colors(0, 20, 0, 0.7)
			else:
				link_colors = self._set_link_colors(20, 0, 0, 0.7)

		print(selected_inv_sol)

		drs = moveit_msgs.msg.DisplayRobotState()
		rs = moveit_msgs.msg.RobotState()

		rs.joint_state.header.frame_id = FRAME_ID
		rs.joint_state.name = JOINTS
		rs.joint_state.position = selected_inv_sol
		rs.multi_dof_joint_state.header.frame_id = FRAME_ID
		rs.is_diff = True
		if self.aco is not None:
			rs.attached_collision_objects = [self.aco]

		drs.state = rs
		drs.highlight_links = link_colors
		self.robot_state_pub.publish(drs)

		return valid, selected_inv_sol

def main():
	rospy.init_node('ur5_inv_kin_wrapper')
	
	ur5_inv = ur5_inv_kin_wrapper()

	listener = tf.TransformListener()
	
	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('/base_link', REAL_EEF_LINK, rospy.Time(0))
			ur5_inv.solve(trans, rot)
			ur5_inv.publish_state(0)
		except Exception as e:
			print(e)
			continue
		print('='*50)
		rospy.Rate(1).sleep()	

if __name__ == '__main__':
	main()