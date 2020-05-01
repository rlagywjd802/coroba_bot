#!/usr/bin/env python
import rospy

from std_msgs.msg import Bool, String
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import *
from moveit_msgs.msg import PlanningScene


from std_srvs.srv import Trigger, TriggerResponse

from const import *

class PointCloudGUI():
	def __init__(self):
		rospy.init_node("pcl_capture")
		
		# Subscriber
		rospy.Subscriber("/pcl_capture", Bool, self.pcl_capture_cb)
		# rospy.Subscriber("/pcl_clear", Bool, self.pcl_clear_cb)	
		# rospy.Subscriber("/move_group/monitored_planning_scene", PlanningScene, self.octomap_update_cb)

		# Publisher
		self.captured_pcl_pub = rospy.Publisher('/captured_pcl', PointCloud2, queue_size=1)
		self.fake_point_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
		self.instruction_pub = rospy.Publisher('/instruction', String, queue_size=1)

		# Service
		rospy.Service("/pcl_clear", Trigger, self.pcl_clear_req)

		# Parameter
		if not rospy.has_param("~captured_cloud_in"):
			rospy.set_param("~captured_cloud_in", "/pcl_fusion_node_color/pcl_fusion_node/fused_points")
		self.pcl_in = rospy.get_param("~captured_cloud_in")

		self.last_pcl = None
		self.last_pcl_len = 0

		self.octomap_updated = False

	# def octomap_update_cb(self, msg):
	# 	if len(msg.world.octomap.octomap.data) > 6:
	# 		self.octomap_updated = True

	# def publish_until_octomap_updated(self, pcl_in, timeout=5.0):
	# 	start = rospy.get_time()
	# 	seconds = rospy.get_time()
	# 	count = 0
	# 	while (seconds - start < timeout) and not rospy.is_shutdown():
	# 		if self.octomap_updated:
	# 			rospy.loginfo("octomap updated")
	# 			self.octomap_updated = False
	# 			return True
	# 		else:
	# 			if count % 5 == 0:
	# 				pcl_in.header.stamp = rospy.Time.now()
	# 				self.captured_pcl_pub.publish(pcl_in)
	# 		rospy.sleep(0.1)
	# 		seconds = rospy.get_time()
	# 		count += 1
	# 	return False
			
	def pcl_capture_cb(self, msg):
		if msg.data:
			rospy.loginfo("clicked pcl capture | {}".format(self.pcl_in))
			
			captured_pcl = rospy.wait_for_message(self.pcl_in, PointCloud2, 60)			
			# if not self.publish_until_octomap_updated(captured_pcl):
			# 	rospy.logerr("failed octomap update")
			captured_pcl.header.stamp = rospy.Time.now()
			self.captured_pcl_pub.publish(captured_pcl)
			
			self.last_pcl = captured_pcl
		else:
			if self.last_pcl is not None:
				last_pcl = self.last_pcl
				last_pcl.header.stamp = rospy.Time.now()
				self.captured_pcl_pub.publish(last_pcl)
				# if not self.publish_until_octomap_updated(last_pcl):
					# rospy.logerr("failed octomap update ")

	def pcl_clear_req(self, req):
		rospy.loginfo("clicked pcl clear")
		clear_cb = PointCloud2()
		clear_cb.header.stamp = rospy.Time.now()
		clear_cb.header.frame_id = 'base_footprint'
		clear_cb.width = 1
		clear_cb.height = 1
		pf_x = PointField(name="x", offset=0, datatype=7, count=1)
		pf_y = PointField(name="y", offset=4, datatype=7, count=1)
		pf_z = PointField(name="z", offset=8, datatype=7, count=1)
		pf_rgb = PointField(name="rgb", offset=16, datatype=7, count=1)
		clear_cb.fields = [pf_x, pf_y, pf_z, pf_rgb]
		clear_cb.is_bigendian = False
		clear_cb.point_step = 32
		clear_cb.row_step = 32
		clear_cb.data = [0] * 32
		clear_cb.is_dense = True

		self.captured_pcl_pub.publish(clear_cb)

		resp = TriggerResponse()
		resp.success = True
		return resp
		# fake_point = PointStamped()
		# fake_point.header.frame_id = FRAME_ID
		# fake_point.point.z = 10 
		# self.fake_point_pub.publish(fake_point) 

if __name__ == '__main__':
	pcl = PointCloudGUI()
	rospy.spin()
