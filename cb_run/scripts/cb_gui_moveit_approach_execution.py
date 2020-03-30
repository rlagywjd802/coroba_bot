#!/usr/bin/env python
import sys
import copy
import rospy
import tf
import math
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

from ur5_inv_kin_wrapper import ur5_inv_kin_wrapper
from utils import *
from const import *

from moveit_commander.conversions import list_to_pose_stamped

# const
DXYZ = 0.02
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
initial_joint_for_capture = d2r([-180.00, -74.67, -91.72, -82.35, 89.65, -270.21])
initial_joint_for_scan = d2r([-180.00, -74.66, -91.72, -103.59, 89.65, -270.21])
initial_quat_for_scan = Quaternion(x=-0.705435194018, y=-0.708766915, z=0.0012030846429, w=0.00303312227606)
# pre_grasp_wand = [-0.234, -0.234, 0.98] # /base_footprint --> /tcp_gripper_closed
wand_grasp_position = [-0.234, -0.154, 0.98] # /base_footprint --> /tcp_gripper_closed


# sanitize_quat = Quaternion(x=-0.705435194018, y=-0.708766915, z=0.0012030846429, w=0.00303312227606)

def dxyz_wrt_eef_pose(cur_pose, dxyz):
    cur_pose_mat = pose_to_mat(cur_pose)
    dpose_mat = tf.transformations.translation_matrix(dxyz)
    pose_mat = tf.transformations.concatenate_matrices(cur_pose_mat, dpose_mat)
    pose = mat_to_pose(pose_mat)

    return pose

def dxyz_wrt_eef_tr(trans, rot, dxyz):
    cur_pose_mat = tr_to_mat(trans, rot)
    dpose_mat = tf.transformations.translation_matrix(dxyz)
    pose_mat = tf.transformations.concatenate_matrices(cur_pose_mat, dpose_mat)
    (trans, mat) = mat_to_tr(pose_mat)

    return trans, mat

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
    out : list of Pose
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
    t_ur5_base = tr_to_mat([-0.000, 0.175, -0.675], [0.000, 0.000, -0.707, 0.707])
    pose_wrt_ur5 = mat_to_pose(tf.transformations.concatenate_matrices(t_ur5_base, pose_wrt_base))
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

        rospy.Subscriber("compute_interpolation", Bool, self.compute_interpolation_cb)
        rospy.Subscriber("execute_interpolation", Bool, self.execute_interpolation_cb)

        rospy.Subscriber("waypoints/update", InteractiveMarkerUpdate, self.waypoints_update_cb)
        rospy.Subscriber("distance", Int32, self.distance_cb)

        rospy.Subscriber("solution_num", Int32, self.solution_cb)

        # rospy.Subscriber("gripper_close", Bool, self.gripper_close_cb)

        rospy.Subscriber("pcl_record", Bool, self.record_move_cb)
        rospy.Subscriber("compute_interpolation", Bool, self.stitch_plan_cb)

        rospy.Subscriber("/cb_gui_moveit/grasp_wand", Bool, self.grasp_wand_cb)
        rospy.Subscriber("/cb_gui_moveit/scanning_plan", Bool, self.scanning_plan_req)
        rospy.Subscriber("/cb_gui_moveit/scanning_execute", Bool, self.scanning_execute_req)

        # Publisher
        self.instruction_pub = rospy.Publisher('/instruction', String, queue_size=1)

        self.aco_pub = rospy.Publisher('/attached_collision_object', moveit_msgs.msg.AttachedCollisionObject, queue_size=100)
        self.captured_pcl = rospy.Publisher('/pcl_capture', Bool, queue_size=1)

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
        self.initialize(initial_joint_for_capture)
        self.scan_quat = None
        self.grasp_wand = False

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

    def move_dxyz(self, dxyz):
        (trans, rot) = self.listener.lookupTransform('/base_link', '/real_ee_link', rospy.Time(0))
        rospy.loginfo("trans:{}, rot:{}".format(trans, rot))

        (dtrans, drot) = dxyz_wrt_eef_tr(trans, rot, dxyz)

        current_joint = self.group.get_current_joint_values()
        self.ur5_inv.solve_and_sort(dtrans, drot, current_joint)

        valid, target_joint = self.ur5_inv.publish_state(0)
        rospy.sleep(1)
        if valid:
            self.move_to_target_joint(target_joint)

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
    def initialize(self, initial_joint):
        self.group.set_end_effector_link("tcp_gripper_closed")
        rospy.loginfo("*"*70)
        rospy.loginfo("end effector: {}".format(self.group.get_end_effector_link()))
        rospy.loginfo("*"*70)

        print "move to initial joint: ", self.move_to_target_joint(initial_joint)

    def grasp_wand_cb(self, msg):
        # all hard coded
        # 0. current -> initial_joint
        # 1. initial_joint_for_scan -> pre-grasp
        # 2. pre-grasp->grasp
        # 3. gripper close
        # 4. attach wand
        # 5. grasp->post-grasp
        # 6. post-grasp->ready_for_scan
        initial_joint = initial_joint_for_scan
        
        print "#"*80 
        print initial_joint
        raw_input()
        print "0. move to initial joint: ", self.move_to_target_joint(initial_joint)

        target_joint = self.group.get_current_joint_values()
        target_joint[5] += math.pi 
        print target_joint
        raw_input()
        print "1. move to target joint: ", self.move_to_target_joint(target_joint)

        target_pose = self.group.get_current_pose()
        target_pose.pose.position.x = wand_grasp_position[0] 
        target_pose.pose.position.y = wand_grasp_position[1]
        target_pose.pose.position.z = wand_grasp_position[2] + 0.08
        print target_pose
        raw_input()
        print "2. move to target pose: ", self.move_to_target_pose(target_pose)

        # move_dxyz(offset, eef_link, axis) move based on target frame -- need to change
        target_pose = self.group.get_current_pose()
        target_pose.pose.position.z -= 0.08
        print target_pose
        raw_input()
        print "3. move to target pose: ", self.move_to_target_pose(target_pose)

        raw_input()
        print "4. grasp the wand:"
        self.gripper_close_pub.publish(Bool(data=True))
        self.attach_uv_wand()

        target_pose = self.group.get_current_pose()
        # target_pose.pose.position.y -= 0.08
        target_pose.pose.position.z += 0.08
        print target_pose
        raw_input()
        print "5. move to target pose: ", self.move_to_target_pose(target_pose)

        target_joint = initial_joint
        target_joint[5] += math.pi
        print target_joint
        raw_input()
        print "6. move to target joint: ", self.move_to_target_joint(target_joint)

        target_joint = self.group.get_current_joint_values()
        target_joint[5] -= math.pi
        print target_joint
        raw_input()
        

        moveit_error = self.move_to_target_joint(target_joint)
        print "7. move to target joint: ", moveit_error
        if moveit_error is None:
            print "Ready for Scanning"
            self.grasp_wand = True


    # def scanning_plan_req(self, req):
    #     '''
    #     1) read csv
    #     2) IK for first pose -> first joint
    #     3) plan for first joint
    #     '''
    #     cvrg_waypoints = read_csv()
    #     # print cvrg_waypoints
    #     if cvrg_waypoints:
    #         self.cvrg_waypoints = cvrg_waypoints
    #         rospy.loginfo("cvrg_waypoints length is {}".format(len(cvrg_waypoints)))
    #         # plan for the first pose
    #         current_joint = self.group.get_current_joint_values()
    #         start_pose = convert_ref_frame(cvrg_waypoints[0])
    #         print "-------start pose is {}".format(start_pose)
    #         trans, rot = pose_to_tr(start_pose)
    #         self.ur5_inv.solve_and_sort(trans, rot, current_joint)
    #         valid, first_joint, first_rs = self.ur5_inv.publish_state(0)
    #         if valid:
    #             # plan for first pose
    #             self.scanning_plan1 = self.plan_for_joint_target(first_joint)
    #             # plan for cvrg_path
    #             self.group.set_start_state(first_rs)
    #             (plan2, fraction) = self.group.compute_cartesian_path(self.cvrg_waypoints, self.eef_step, self.jump_threshold)
    #             rospy.loginfo("compute_interpolation_cb| Visualizing Cartesian path ({:.2f} per acheived)".format(fraction*100.0));
    #             self.scanning_plan2 = plan2
    #             return True
    #         else:
    #             rospy.logerror("first pose IK is not valid")
    #             return False
    #     else:
    #         return False
    def attach_uv_wand(self):
        attach_link = "tcp_gripper_closed"
        part_name = "uv_wand"
        part_size = (0.27, 0.05, 0.05)    # box [26, 4, 4] + padding 1
        part_pose = geometry_msgs.msg.PoseStamped()
        part_pose.header.frame_id = "tcp_gripper_closed"
        part_pose.header.stamp = rospy.Time.now()
        part_pose.pose.position.x = -0.1
        touch_links = self.robot.get_link_names(group='robotiq')
        print "attach uv_wand|", self.scene.attach_box(link=attach_link, name=part_name, pose=part_pose, size=part_size, touch_links=touch_links)
        self.group.set_end_effector_link("uv_light_center")
        rospy.loginfo("*"*70)
        rospy.loginfo("end effector: {}".format(self.group.get_end_effector_link()))
        rospy.loginfo("*"*70)


    def scanning_plan_req(self, req):
        '''
        1) read csv
        2) IK for first pose -> first joint
        3) plan for first joint
        '''
        if self.grasp_wand:
            scan_pose = self.group.get_current_pose()
            scan_quat = scan_pose.pose.orientation
            print "quaternion for scan: ", scan_quat   

            cvrg_waypoints = read_csv(scan_quat)
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

    def scanning_execute_req(self, req):
        print "scanning execute1|", self.group.execute(self.scanning_plan1)
        print "current joint:\n{}".format(self.group.get_current_joint_values())
        print "inital joint of execute2:\n{}".format(self.scanning_plan2.joint_trajectory.points[0].positions)
        print "scanning execute|",self.move_to_target_joint(self.scanning_plan2.joint_trajectory.points[0].positions)
        print "scanning execute2|", self.group.execute(self.scanning_plan2)

    def compute_interpolation(self):
        rospy.loginfo("compute_interpolation_cb|")
        if self.cvrg_waypoints:
            # compute cartesian path
            (path, fraction) = self.group.compute_cartesian_path(self.cvrg_waypoints, self.eef_step, self.jump_threshold)
            rospy.loginfo("compute_interpolation_cb| Visualizing Cartesian path ({:.2f} per acheived)".format(fraction*100.0));
            self.traj_for_wpts = path
        else:
            rospy.logerr("compute_interpolation_cb| No Waypoints")

    def compute_interpolation_cb(self, msg):
        rospy.loginfo("compute_interpolation_cb|")
        cur_pose = self.group.get_current_pose().pose
        pp = cur_pose.position
        po = cur_pose.orientation
        poe = tf.transformations.euler_from_quaternion([po.x, po.y, po.z, po.w])
        rospy.loginfo("current pose: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}"
                            .format(pp.x, pp.y, pp.z, poe[0], poe[1], poe[2]))

        if self.cvrg_waypoints:
            # compute cartesian path
            (path, fraction) = self.group.compute_cartesian_path(self.cvrg_waypoints, self.eef_step, self.jump_threshold)
            rospy.loginfo("compute_interpolation_cb| Visualizing Cartesian path ({:.2f} per acheived)".format(fraction*100.0));
            self.traj_for_wpts = path
        else:
            rospy.logerr("compute_interpolation_cb| No Waypoints")
    
    def execute_interpolation_cb(self, msg):
        rospy.logdebug("execute_interpolation_cb")
        if self.traj_for_wpts:
            self.group.execute(self.traj_for_wpts)
        else:
            rospy.logerr("execute_interpolation_cb| No Trajectory")

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
                (trans, rot) = self.listener.lookupTransform('/base_link', REAL_EEF_LINK, rospy.Time(0))
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

    # def gripper_close_cb(self, msg):
    #     if msg.data:
    #         rospy.sleep(2)  # wait for close gripper
    #         attach_link = "left_inner_finger_pad"
    #         part_name = "part"
    #         part_size = 0.1 # sphere - radius
    #         # part_size = (0.12, 0.02, 0.15)    # box
    #         part_pose = geometry_msgs.msg.PoseStamped()
    #         part_pose.header.frame_id = "left_inner_finger_pad"
    #         part_pose.header.stamp = rospy.Time.now()
    #         part_pose.pose.position.y = -0.01
    #         part_pose.pose.position.z = 0.05
    #         touch_links = self.robot.get_link_names(group='robotiq')
    #         # self.scene.attach_box(link=attach_link, name=part_name, pose=part_pose, size=part_size, touch_links=touch_links)
    #         self.attach_sphere(attach_link, part_name, part_pose, part_size, touch_links)
    #         self.captured_pcl.publish(Bool(False))
    #     else:
    #         self.scene.remove_attached_object("left_inner_finger_pad", "part")
    #         rospy.sleep(1)
    #         self.scene.remove_world_object("part")
    #         self.captured_pcl.publish(Bool(False))

def main(arg):
    if len(arg) > 1:
        if arg[1] == "debug":
            log_level = rospy.DEBUG
    else:
        log_level = rospy.INFO

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