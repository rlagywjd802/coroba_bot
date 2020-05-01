#!/usr/bin/env python
import sys
import copy
import rospy
import tf
import math as m
import csv

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import shape_msgs.msg
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose, PointStamped, PoseStamped
from visualization_msgs.msg import InteractiveMarkerUpdate
from sensor_msgs.msg import Joy

from std_srvs.srv import Trigger, TriggerResponse, Empty
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest
from ur5_inv_kin_wrapper import ur5_inv_kin_wrapper
from utils import *
from const import *

from moveit_commander.conversions import list_to_pose_stamped

# const
DXYZ = 0.02
DJ = m.pi/12.0
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# initial_joint_for_capture = d2r([-180.00, -74.67, -91.72, -82.35, 89.65, -270.21])
initial_joint_for_capture = d2r([-180.00, -74.67, -91.72, -65.57, 89.65, -270.21])
initial_joint_for_scan = d2r([-180.00, -74.66, -91.72, -103.59, 89.65, -270.21])
#initial_joint_for_scan = d2r([-86.42, -121.81, -93.17, 34.85, 81.88, -270.01])
quat_for_scan = quat_for_wand = euler_to_quat(-m.pi, 0.0, m.pi/2.0)
# initial_quat_for_scan = Quaternion(x=-0.705435194018, y=-0.708766915, z=0.0012030846429, w=0.00303312227606)
# pre_grasp_wand = [-0.234, -0.234, 0.98] # /base_footprint --> /tcp_gripper_closed
wand_grasp_position = [-0.234, -0.154, 0.98] # /base_footprint --> /tcp_gripper_closed


# def dxyz_wrt_eef_pose(cur_pose, dxyz):
#     cur_pose_mat = pose_to_mat(cur_pose) # /REF_LINK -> /EEF_LINK
#     dpose_mat = tf.transformations.translation_matrix(dxyz)
#     pose_mat = tf.transformations.concatenate_matrices(cur_pose_mat, dpose_mat)
#     pose = mat_to_pose(pose_mat)

#     return pose

def dxyz_wrt_eef_pose(cur_pose, dxyz):
    cur_pose_mat = pose_to_mat(cur_pose) # /REF_LINK -> /EEF_LINK
    dpose_mat = tf.transformations.translation_matrix(dxyz)
    pose_mat = tf.transformations.concatenate_matrices(cur_pose_mat, dpose_mat)
    (trans, mat) = mat_to_tr(pose_mat)

    return (trans, mat)

def dxyz_wrt_eef_tr(trans, rot, dxyz):
    cur_pose_mat = tr_to_mat(trans, rot)
    dpose_mat = tf.transformations.translation_matrix(dxyz)
    pose_mat = tf.transformations.concatenate_matrices(cur_pose_mat, dpose_mat)
    (trans, mat) = mat_to_tr(pose_mat)

    return trans, mat

# def clear_octomap():
#     rospy.wait_for_service("/apply_planning_scene")
#     try:
#         apc_service_call = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
#         apcr = ApplyPlanningSceneRequest()
#         # apcr.scene.name = 'disinfection_scene'
#         apcr.scene.is_diff = True
#         apcr.scene.world.octomap.header.frame_id = "/base_footprint"
#         apcr.scene.world.octomap.octomap.id = 'OcTree'
#         apcr.scene.world.octomap.octomap.data = [0.0]
#         apcr.scene.world.octomap.octomap.data = [0.0]
#         resp = apc_service_call(apcr)
#         if resp.success:
#             rospy.logdebug("clear_octomap| success")
#             return True
#         else:
#             return False
#     except Exception as e:
#         rospy.logerr(e)
#         return False

def clear_octomap():
    rospy.wait_for_service("/clear_octomap")
    try:
        clear_octomap_call = rospy.ServiceProxy('/clear_octomap', Empty)
        apcr = ApplyPlanningSceneRequest()
        resp = clear_octomap_call()
        return True
    except Exception as e:
        rospy.logerr(e)
        return False


def pcl_fusion_reset():
    try:
        reset_pcl = rospy.ServiceProxy('/pcl_fusion_node/reset', Trigger)
        resp = reset_pcl()
        if resp.success:
            rospy.loginfo("pcl_fusion_reset| reset success")
            return True
        else:
            return False
    except Exception as e:
        rospy.loginfo(e)
        return False

def read_csv(quat):
    '''
    set every waypoints' orientation as quat
    return : list of Pose
    '''
    rospy.loginfo("read csv")
    try: 
        csv_path = rospy.get_param("/cvrg_file_paths/cvrg_path")
        with open(csv_path) as csvfile:      
            wpt_reader = csv.reader(csvfile, delimiter=",")
            wpt_list = []
            for row in wpt_reader:
                row_float = [float(e) for e in row]
                ps = Pose()
                ps.position.x = row_float[0]
                ps.position.y = row_float[1]
                ps.position.z = row_float[2]
                ps.orientation = quat
                wpt_list.append(ps)
            return wpt_list
    except Exception as e:
        rospy.logerr(e)
        return []

def scan_from_nearest(wpts, cur_pose):
    '''
    wpts : list of Pose
    cur_pose : Pose
    '''
    def dist_btw_point(p1, p2):
        dist = m.sqrt((p1.x-p2.x)**2+(p1.y-p2.y)**2+(p1.z-p2.z)**2)
        return dist
    dist_first = dist_btw_point(wpts[0].position, cur_pose.position)
    dist_last = dist_btw_point(wpts[-1].position, cur_pose.position)
    print "dist to first and last: {}, {}".format(dist_first, dist_last)
    if dist_first > dist_last:
        print "reversed"
        reversed_wpts = wpts[::-1]
        return reversed_wpts
    else:
        return wpts

def convert_ref_frame(pose):
    '''
    in : [position, orien(euler)]
    convert tf from /base_footprint -> /real_ee_link : p1
                to /real_base_link -> /real_ee_link  : p0
                p0 = T01*p1
    '''
    # start_pose = convert_ref_frame(cvrg_waypoints[0])
    # trans, rot = pose_to_tr(start_pose)
    # trans, rot = convert_ref_frame(cvrg_waypoints[0])
    pose_wrt_base = pose_to_mat(pose)
    T_ur5_base = tr_to_mat([-0.000, 0.175, -0.675], [0.000, 0.000, -0.707, 0.707])
    pose_wrt_ur5 = mat_to_pose(tf.transformations.concatenate_matrices(T_ur5_base, pose_wrt_base))
    return pose_wrt_ur5

def test_convert_tf():
    current.position.x = 0.347176858326
    current.position.y = 0.468640011366
    current.position.z = 1.14603542809
    current.orientation.x = 0.765530197885
    current.orientation.y = -0.643049350175
    current.orientation.z = 0.0140599077307
    current.orientation.w = 0.015917548841

    converted_pose = convert_ref_frame([0.347176858326, 0.468640011366, 1.14603542809, 0.765530197885, -0.643049350175, 0.0140599077307, 0.015917548841])
    print converted_pose

class UR5MoveGroupGUI():
    def __init__(self, log_level):
        rospy.init_node('cb_moveit_gui_execution', log_level=log_level)

        # Subscriber
        # rospy.Subscriber("clicked_point", PointStamped, self.clicked_cb)

        rospy.Subscriber("pick_approach_plan", Bool, self.pick_approach_plan_cb)
        rospy.Subscriber("pick_approach_execute", Bool, self.pick_approach_execute_cb)
        rospy.Subscriber("pick_retreat_plan", Bool, self.pick_retreat_plan_cb)
        rospy.Subscriber("pick_retreat_execute", Bool, self.pick_retreat_execute_cb)
        rospy.Subscriber("place_approach_plan", Bool, self.place_approach_plan_cb)
        rospy.Subscriber("place_approach_execute", Bool, self.place_approach_execute_cb)
        rospy.Subscriber("place_retreat_plan", Bool, self.place_retreat_plan_cb)
        rospy.Subscriber("place_retreat_execute", Bool, self.place_retreat_execute_cb)

        rospy.Subscriber("approach_stop", Bool, self.approach_stop_cb)

        rospy.Subscriber("move_xp", Bool, self.move_xp_cb)
        rospy.Subscriber("move_xm", Bool, self.move_xm_cb)
        rospy.Subscriber("move_yp", Bool, self.move_yp_cb)
        rospy.Subscriber("move_ym", Bool, self.move_ym_cb)
        rospy.Subscriber("move_zp", Bool, self.move_zp_cb)
        rospy.Subscriber("move_zm", Bool, self.move_zm_cb)

        rospy.Subscriber("waypoints/update", InteractiveMarkerUpdate, self.waypoints_update_cb)
        rospy.Subscriber("distance", Int32, self.distance_cb)

        rospy.Subscriber("solution_num", Int32, self.solution_cb)

        rospy.Subscriber("pcl_record", Bool, self.record_move_cb)
        rospy.Subscriber("compute_interpolation", Bool, self.stitch_plan_cb)

        rospy.Subscriber("/cb_gui_moveit/pcl_capture", Bool, self.pcl_capture_cb)
        rospy.Subscriber("/cb_gui_moveit/pcl_clear", Bool, self.pcl_clear_cb)

        rospy.Subscriber("/cb_gui_moveit/grasp_object", Bool, self.grasp_object_cb)
        rospy.Subscriber("/cb_gui_moveit/grasp_wand", Bool, self.grasp_wand_cb)
        
        rospy.Subscriber("/cb_gui_moveit/scanning_plan", Bool, self.scanning_plan_req)
        rospy.Subscriber("/cb_gui_moveit/scanning_execute", Bool, self.scanning_execute_req)

        rospy.Subscriber("/cb_gui_moveit/j1", Bool, self.move_j1_cb)
        rospy.Subscriber("/cb_gui_moveit/j4", Bool, self.move_j4_cb)
        rospy.Subscriber("/cb_gui_moveit/j6", Bool, self.move_j6_cb)

        # Publisher
        self.instruction_pub = rospy.Publisher('/instruction', String, queue_size=1)

        self.aco_pub = rospy.Publisher('/attached_collision_object', moveit_msgs.msg.AttachedCollisionObject, queue_size=100)
        
        self.pcl_capture_pub = rospy.Publisher('/pcl_capture', Bool, queue_size=1)

        self.remove_imarker_pub = rospy.Publisher("/remove_imarker", Bool, queue_size=1)

        self.gripper_close_pub = rospy.Publisher("/gripper_close", Bool, queue_size=1)

        # Service
        rospy.wait_for_service('/pcl_fusion_node/reset')
        rospy.Service("/cb_gui_moveit/scanning_plan", Trigger, self.scanning_plan_req)
        rospy.Service("/cb_gui_moveit/scanning_execute", Trigger, self.scanning_execute_req)

        # ready for listen tf
        self.listener = tf.TransformListener()

        # moveit commander
        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.group = moveit_commander.MoveGroupCommander("ur5")         
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(10)
        self.group.set_max_velocity_scaling_factor(0.05)        # 0.1
        self.group.set_max_acceleration_scaling_factor(0.05)    # 0.1

        self.pick_approach_plan = None
        self.pick_retreat_plan = None
        self.pick_retreat_plan1 = None
        self.place_approach_plan = None
        self.place_retreat_plan = None

        # for compute cartesian path
        self.scanning_plan1 = None
        self.scanning_plan2 = None
        self.jump_threshold = 0.0
        self.eef_step = 0.01

        # set after execution
        self.pre_grasp_joint = None
        self.post_grasp_joint = None

        self.pick_retreat_step = 0

        self.last_offset = 10.0

        # IK
        self.ur5_inv = ur5_inv_kin_wrapper()
        self.last_selected_joint = None
        self.last_sol_num = None

        self.count = 0

        # initialize
        # from capture
        #self.initialize(initial_joint_for_capture)
        #self.grasp_wand = False
        # from scan
        self.initialize(initial_joint_for_scan)
        self.grasp_wand = True

    ##################################################################################################
    ##################################################################################################

    def move_to_target_joint(self, joint):
        self.group.set_joint_value_target(joint)
        self.group.go(wait=True)
        self.group.clear_pose_targets()
        rospy.sleep(1)

    def move_to_target_pose(self, pose):
        self.group.set_pose_target(pose)
        self.group.go(wait=True)
        self.group.clear_pose_targets()

    # def move_dxyz(self, dxyz):
    #     '''
    #     dxyz = [dx, dy, dz]
    #     '''
    #     print "initialize planning setting"
    #     self.group.set_planning_time(1.0)
    #     self.group.set_num_planning_attempts(3)

    #     current_pose = self.group.get_current_pose()
    #     target_pose = dxyz_wrt_eef_pose(current_pose.pose, dxyz)
    #     print "move_dxyz error: ", self.move_to_target_pose(target_pose)

    #     print "revert planning setting"
    #     self.group.set_planning_time(5.0)
    #     self.group.set_num_planning_attempts(10)

    def move_dxyz(self, dxyz):
        '''
        dxyz = [dx, dy, dz]
        '''
        print "initialize planning setting"
        self.group.set_planning_time(1.0)
        self.group.set_num_planning_attempts(3)
        self.set_end_effector(link="real_ee_link")

        current_pose = self.group.get_current_pose()
        print current_pose

        current_pose_wrt_ur5 = convert_ref_frame(current_pose.pose)
        (dtrans, drot) = dxyz_wrt_eef_pose(current_pose_wrt_ur5, dxyz)
        current_joint = self.group.get_current_joint_values()
        self.ur5_inv.solve_and_sort(dtrans, drot, current_joint)

        valid, target_joint = self.ur5_inv.publish_state(0)
        if valid:
            self.move_to_target_joint(target_joint)

        print "revert planning setting"
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(10)


    def plan_for_joint_target(self, joint):
        rospy.loginfo("plan to joint target| started")
        self.group.set_joint_value_target(joint)
        plan = self.group.plan()
        self.group.clear_pose_targets()
        rospy.loginfo("plan to joint target| finished")
        return plan

    def plan_for_pose_target(self, pose):
        rospy.loginfo("plan to pose target| started")
        self.group.set_pose_target(pose)
        plan = self.group.plan()
        self.group.clear_pose_targets()
        rospy.loginfo("plan to pose target| finished")
        return plan

    def execute_plan(self, plan):
        rospy.loginfo("execute plan| started")
        self.group.execute(plan, wait=True)
        # rospy.sleep(5)
        rospy.loginfo("execute plan| finished")

    def attach_sphere(self, link, name, pose, radius, touch_links):
        aco = moveit_msgs.msg.AttachedCollisionObject()
        aco.object = self.make_sphere(name, pose, radius) ##
        aco.link_name = link
        aco.touch_links = touch_links
        self.aco_pub.publish(aco)

    def make_sphere(self, name, pose, radius):
        co = moveit_msgs.msg.CollisionObject()
        co.operation = moveit_msgs.msg.CollisionObject.ADD
        co.id = name
        co.header = pose.header
        sphere = shape_msgs.msg.SolidPrimitive()
        sphere.type = shape_msgs.msg.SolidPrimitive.SPHERE
        sphere.dimensions = [radius]
        co.primitives = [sphere]
        co.primitive_poses = [pose.pose]
        return co

    ##################################################################################################
    ##################################################################################################
    def pcl_capture_cb(self, msg):
        self.pcl_capture_pub.publish(Bool(True))

    def pcl_clear_cb(self, msg):
        rospy.wait_for_service("/pcl_clear")
        try:
            pcl_clear_srv_call = rospy.ServiceProxy('/pcl_clear', Trigger)
            resp = pcl_clear_srv_call()
            if resp.success:
                is_cleared = clear_octomap()
                rospy.loginfo("pcl_clear_cb| octomap cleared {}".format(is_cleared))
            else:
                rospy.logerr("pcl_clear_cb| pcl clear srv call failed")
        except Exception as e:
            rospy.logerr(e)

    def set_end_effector(self, link, info=True):
        self.group.set_end_effector_link(link)
        if info:
            rospy.loginfo("*"*70)
            rospy.loginfo("end effector: {}".format(self.group.get_end_effector_link()))
            rospy.loginfo("*"*70)

    def wait_for_scene_update(self, name, is_known=False, is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (is_attached == is_attached) and (is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_uv_wand_case(self):
        box_name = "uv_wand_case"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_footprint"
        box_pose.pose.position.x = wand_grasp_position[0]
        box_pose.pose.position.y = wand_grasp_position[1] + 0.1
        box_pose.pose.position.z = wand_grasp_position[2] - 0.01
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, m.pi/2.0)
        box_pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        box_size = (0.20, 0.06, 0.06)
        
        self.scene.add_box(name=box_name, pose=box_pose, size=box_size)
        is_updated = self.wait_for_scene_update(name=box_name, is_known=True)
        rospy.loginfo("add uv_wand_case|{}".format(is_updated))

    def remove_uv_wand_case(self):
        box_name = "uv_wand_case"

        self.scene.remove_world_object(box_name)
        is_updated = self.wait_for_scene_update(name=box_name, is_attached=False, is_known=False)
        rospy.loginfo("remove uv_wand_case|{}".format(is_updated))

    def attach_uv_wand(self):
        attach_link = "tcp_gripper_closed"
        part_name = "uv_wand"
        part_size = (0.27, 0.05, 0.05)    # box [26, 4, 4] + padding 1
        part_pose = geometry_msgs.msg.PoseStamped()
        part_pose.header.frame_id = "tcp_gripper_closed"
        part_pose.header.stamp = rospy.Time.now()
        part_pose.pose.position.x = -0.1
        touch_links = self.robot.get_link_names(group='robotiq')
        
        self.scene.attach_box(link=attach_link, name=part_name, pose=part_pose, size=part_size, touch_links=touch_links)
        is_updated = self.wait_for_scene_update(name=part_name, is_known=True)
        rospy.loginfo("attach uv_wand|{}".format(is_updated))
    
        # update pcl
        self.pcl_capture_pub.publish(Bool(False))
        # change end effector
        # self.set_end_effector(link="uv_light_center")

    def detach_uv_wand(self):
        attach_link = "tcp_gripper_closed"
        part_name = "uv_wand"

        self.scene.remove_attached_object(attach_link, part_name)
        self.wait_for_scene_update(name=part_name, is_known=True, is_attached=False)
        self.scene.remove_world_object(part_name)
        is_updated = self.wait_for_scene_update(name=part_name, is_attached=False, is_known=False)
        rospy.loginfo("detach uv_wand|{}".format(is_updated))

        # update pcl
        self.pcl_capture_pub.publish(Bool(False))
        # change end effector
        # self.set_end_effector(link="tcp_gripper_closed")

    def initialize(self, initial_joint):
        self.set_end_effector(link="tcp_gripper_closed")
        #print "move to initial joint: ", self.move_to_target_joint(initial_joint)

    def grasp_wand_cb(self, msg):
        '''
        eef_link : /tcp_gripper_closed 
        '''
        self.set_end_effector(link="tcp_gripper_closed")
        self.group.set_max_velocity_scaling_factor(0.1)        # 0.1
        self.group.set_max_acceleration_scaling_factor(0.1)    # 0.1

        print "#"*80 
        # target_pose = self.group.get_current_pose()
        # target_pose.pose.orientation = quat_for_wand 
        # print "----------> press 'Enter':", raw_input()
        # print "0. move to initial quat: ", self.move_to_target_pose(target_pose)        
        # initial_pose = target_pose

        initial_joint = initial_joint_for_scan
        # print "----------> press 'Enter':", raw_input()
        print "0. move to initial joint: ", self.move_to_target_joint(initial_joint)

        target_pose = self.group.get_current_pose()
        target_pose.pose.position.x = wand_grasp_position[0] 
        target_pose.pose.position.y = wand_grasp_position[1]
        target_pose.pose.position.z = wand_grasp_position[2] + 0.08
        print target_pose
        # print "----------> press 'Enter':", raw_input()
        print "1. move to target pose: ", self.move_to_target_pose(target_pose)

        target_joint = self.group.get_current_joint_values()
        target_joint[5] += m.pi 
        print target_joint
        # print "----------> press 'Enter':", raw_input()
        print "2. move to target joint: ", self.move_to_target_joint(target_joint)
        if not msg.data:
            self.remove_uv_wand_case()

        # move_dxyz(offset, eef_link, axis) move based on target frame -- need to change
        target_pose = self.group.get_current_pose()
        target_pose.pose.position.z -= 0.08
        print target_pose
        # print "----------> press 'Enter':", raw_input()
        print "3. move to target pose: ", self.move_to_target_pose(target_pose)

        # print "----------> press 'Enter':", raw_input()
        if msg.data:
            print "4. grasp the wand:"
            self.gripper_close_pub.publish(Bool(data=True))
            self.attach_uv_wand()
            rospy.sleep(2)
        else:
            print "4. put the wand back:"
            self.gripper_close_pub.publish(Bool(data=False))
            self.detach_uv_wand()
            rospy.sleep(2)

        target_pose = self.group.get_current_pose()
        target_pose.pose.position.z += 0.08
        print target_pose
        # print "----------> press 'Enter':", raw_input()
        print "5. move to target pose: ", self.move_to_target_pose(target_pose)
        if msg.data:
            self.add_uv_wand_case()

        target_joint = self.group.get_current_joint_values()
        target_joint[5] -= m.pi
        print target_joint
        # print "----------> press 'Enter':", raw_input()
        print "6. move to target joint: ", self.move_to_target_joint(target_joint)

        # print "----------> press 'Enter':", raw_input()
        print "7. move to initial joint: ", self.move_to_target_joint(initial_joint)

        if msg.data:
            print "Ready for Scanning"
            self.grasp_wand = True
        else:
            print "Ready for Capturing"
            self.grasp_wand = False

        self.group.set_max_velocity_scaling_factor(0.05)        # 0.1
        self.group.set_max_acceleration_scaling_factor(0.05)    # 0.1

    def grasp_object_cb(self, msg):
        attach_link = "tcp_gripper_closed"
        obj_name = "object"
        obj_size = 0.1 # sphere(radius)
        obj_pose = geometry_msgs.msg.PoseStamped()
        obj_pose.header.frame_id = "tcp_gripper_closed"
        obj_pose.header.stamp = rospy.Time.now()
        touch_links = self.robot.get_link_names(group='robotiq')

        if msg.data:
            self.attach_sphere(attach_link, obj_name, obj_pose, obj_size, touch_links)
            is_updated = self.wait_for_scene_update(name=obj_name, is_known=True)
            rospy.loginfo("attach object|{}".format(is_updated))
            self.pcl_capture_pub.publish(Bool(False))
            self.gripper_close_pub.publish(Bool(True))
        else:
            self.scene.remove_attached_object(attach_link, obj_name)      
            self.wait_for_scene_update(name=obj_name, is_known=True, is_attached=False)
            self.scene.remove_world_object(obj_name)
            is_updated = self.wait_for_scene_update(name=obj_name, is_attached=False, is_known=False)
            rospy.loginfo("detach object|{}".format(is_updated))
            self.pcl_capture_pub.publish(Bool(False))
            self.gripper_close_pub.publish(Bool(False))

    def scanning_plan_req(self, req):
        '''
        1) read csv
        2) IK for first pose -> first joint
        3) plan for first joint
        '''
        if self.grasp_wand:
            self.set_end_effector(link="uv_light_center")
            scan_pose = self.group.get_current_pose()
            scan_quat = scan_pose.pose.orientation
            print "quaternion for scan:\n", scan_quat   

            cvrg_waypoints_origin = read_csv(scan_quat)
            cvrg_waypoints = scan_from_nearest(cvrg_waypoints_origin, scan_pose.pose)
            # print "----------> press 'Enter':", raw_input()
            # print "orinal and sorted first wpt:\n{}\n{}".format(cvrg_waypoints_origin[0], cvrg_waypoints[0])
            # print cvrg_waypoints
            if cvrg_waypoints:
                self.cvrg_waypoints = cvrg_waypoints
                rospy.loginfo("cvrg_waypoints length is {}".format(len(cvrg_waypoints)))
                # plan for first pose
                self.scanning_plan1 = self.plan_for_pose_target(cvrg_waypoints[0])
                rs = moveit_msgs.msg.RobotState()
                rs.joint_state.header.frame_id = FRAME_ID
                rs.joint_state.name = JOINTS
                rs.joint_state.position = self.scanning_plan1.joint_trajectory.points[-1].positions
                # plan for cvrg_path
                self.group.set_start_state(rs)
                (plan2, fraction) = self.group.compute_cartesian_path(self.cvrg_waypoints, self.eef_step, self.jump_threshold)
                rospy.loginfo("compute_interpolation_cb| Visualizing Cartesian path ({:.2f} per acheived)".format(fraction*100.0));
                self.scanning_plan2 = plan2
        else:
            rospy.logerror("Didn't grasp wand yet")

    def scanning_execute_req(self, req):
        print "scanning execute1|", self.group.execute(self.scanning_plan1)
        print "current joint:\n{}".format(self.group.get_current_joint_values())
        print "inital joint of execute2:\n{}".format(self.scanning_plan2.joint_trajectory.points[0].positions)
        print "scanning execute|",self.move_to_target_joint(self.scanning_plan2.joint_trajectory.points[0].positions)
        print "scanning execute2|", self.group.execute(self.scanning_plan2)

    ##################################################################################################
    ##################################################################################################

    def move_xp_cb(self, msg):
        rospy.loginfo("move_xp_cb")
        self.move_dxyz([0, 0, DXYZ])

    def move_xm_cb(self, msg):
        rospy.loginfo("move_xm_cb")
        self.move_dxyz([0, 0, -DXYZ])

    def move_yp_cb(self, msg):
        rospy.loginfo("move_yp_cb")
        self.move_dxyz([0, DXYZ, 0])

    def move_ym_cb(self, msg):
        rospy.loginfo("move_ym_cb")
        self.move_dxyz([0, -DXYZ, 0])

    def move_zp_cb(self, msg):
        rospy.loginfo("move_yp_cb")
        self.move_dxyz([-DXYZ, 0, 0])

    def move_zm_cb(self, msg):
        rospy.loginfo("move_ym_cb")
        self.move_dxyz([DXYZ, 0, 0])

    def move_j1_cb(self, msg):
        target_joint = self.group.get_current_joint_values()
        if msg.data:
            target_joint[0] += DJ
        else:
            target_joint[0] -= DJ
        self.move_to_target_joint(target_joint)

    def move_j4_cb(self, msg):
        target_joint = self.group.get_current_joint_values()
        if msg.data:
            target_joint[3] += DJ
        else:
            target_joint[3] -= DJ
        self.move_to_target_joint(target_joint)

    def move_j6_cb(self, msg):
        target_joint = self.group.get_current_joint_values()
        if msg.data:
            target_joint[5] += DJ*6
        else:
            target_joint[5] -= DJ*6
        self.move_to_target_joint(target_joint)

    def pick_approach_plan_cb(self, msg):
        '''
        initial joint -> pre-grasp joint
        '''
        if self.last_selected_joint is not None:
            self.pick_approach_plan = self.plan_for_joint_target(self.last_selected_joint)

    def pick_approach_execute_cb(self, msg):
        if self.pick_approach_plan is not None:
            self.remove_imarker_pub.publish(Bool(True))
            self.execute_plan(self.pick_approach_plan)
            self.pre_grasp_joint = self.group.get_current_joint_values(self.scanning_plan2.joint_trajectory.points[0].positions)
            rospy.loginfo("pick_approach_execute done")


    def pick_retreat_plan_cb(self, msg):
        '''
        grasp joint -> pre-grasp joint -> initial joint
        '''
        if self.pre_grasp_joint is not None:
            self.ur5_inv.publish_state(-1)

            self.pick_retreat_plan = self.plan_for_joint_target(initial_joint)
            # if self.pick_retreat_step == 0:
            #     self.pick_retreat_plan = self.plan_for_joint_target(self.pre_grasp_joint)
            #     rospy.loginfo("pick retreat plan | step 0")
            # elif self.pick_retreat_step == 1:
            #     self.pick_retreat_plan1 = self.plan_for_joint_target(initial_joint)
            #     rospy.loginfo("pick retreat plan | step 1")

    def pick_retreat_execute_cb(self, msg):
        if self.pick_retreat_plan is not None:
            self.execute_plan(self.pick_retreat_plan)
            # if self.pick_retreat_step == 0:
            #     self.execute_plan(self.pick_retreat_plan)
            #     self.pick_retreat_step += 1
            # elif self.pick_retreat_step == 1:
            #     self.execute_plan(self.pick_retreat_plan1)
            #     self.pick_retreat_step += 1

    def place_approach_plan_cb(self, msg):
        pass

    def place_approach_execute_cb(self, msg):
        pass

    def place_retreat_plan_cb(self, msg):
        pass

    def place_retreat_execute_cb(self, msg):
        pass

    def approach_stop_cb(self, msg):
        self.group.stop()
        rospy.loginfo("approach_stop: Stopped")

    def waypoints_update_cb(self, msg):
        '''
        if waypoints/update poses are updated,
        1) solve IK for eef pose of physical robot(real_eef_link) wrt /base_link
        2) sort based on the diff btw current joint
        3) publish lastly selected sol of robot states
            (if sol hasn't seleceted, publish clear state)
        '''
        if msg.poses:
            rospy.loginfo("waypoints_update_cb")
            try:
                (trans, rot) = self.listener.lookupTransform('/real_base_link', REAL_EEF_LINK, rospy.Time(0))
                rospy.loginfo("waypoints_update_cb| trans:{}, rot:{}".format(trans, rot))

                current_joint = self.group.get_current_joint_values()
                self.ur5_inv.solve_and_sort(trans, rot, current_joint)

                if self.last_sol_num is None: 
                    self.ur5_inv.publish_state(-1)
                elif self.last_sol_num is -1:
                    self.ur5_inv.publish_state(-1)
                    self.last_selected_joint = None
                else:
                    _, target_joint = self.ur5_inv.publish_state(self.last_sol_num)
                    self.last_selected_joint = target_joint

            except Exception as e:
                rospy.logerr(e)
                pass

    def record_move_cb(self, msg):
        for i in range(len(record_motion)):
            self.move_to_target_joint(record_motion[i])    
            rospy.loginfo("record_move_cb| joint {}".format(i))
            if i == 0:
                is_reset = pcl_fusion_reset()
                if is_reset == False:   break
        
        rospy.loginfo("record_move_cb| finished")

    def stitch_plan_cb(self, msg):
        self.move_to_target_joint(record_motion[self.count])
        rospy.loginfo("stitch_plan_cb| joint {}".format(self.count))

        if self.count == 0:    
            is_reset = pcl_fusion_reset()

        if (self.count > 0) or (is_reset == True):
            self.count += 1

    def distance_cb(self, msg):
        offset = msg.data

        rospy.loginfo("distance_cb| msg: {}".format(offset))
        self.last_offset = offset

    def solution_cb(self, msg):
        '''
        if sol num is clicked,
        publish selected sol(-1~7) of robot state and save it
        (if -1 is selected, save target joint as None)
        '''
        sol_num = msg.data

        rospy.loginfo("solution_cb| msg: {}".format(sol_num))
        
        _, target_joint = self.ur5_inv.publish_state(sol_num)

        if sol_num == -1:
            self.last_selected_joint = None
        else:
            self.last_selected_joint = target_joint

        self.last_sol_num = sol_num

def main(arg):
    log_level = rospy.INFO
    if len(arg) > 1 and arg[1] == "debug":
            log_level = rospy.DEBUG

    cnt = 0 # publish transparent robot state for few times
    try:
        ur5 = UR5MoveGroupGUI(log_level)
        while not rospy.is_shutdown():
            if cnt < 2:
                ur5.ur5_inv.publish_state(-1)
            cnt += 1
            rospy.sleep(1)

    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main(sys.argv)
