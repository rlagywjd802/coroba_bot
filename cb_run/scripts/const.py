from geometry_msgs.msg import Quaternion

# only ur5
# MOVE_GROUP = "manipulator"
# FRAME_ID = "world"
# REAL_EEF_LINK = "real_ee_link"

# cb
MOVE_GROUP = "ur5"
FRAME_ID = "/base_footprint"
REF_LINK = "/real_base_link"
EEF_LINK = "/real_ee_link"
# EEF_LINK = "eef_pose"
REAL_EEF_LINK = "real_eef_pose"

GRIPPER_MESH = "package://cb_description/meshes/gripper/robotiq_2f85_opened_combined_axis_mated.STL"

INIT_R_AXIS = "z"
INIT_ORIENT = Quaternion(0.0, 0.0, -0.707107, 0.707107) # euler_to_quat(0, 0, -math.pi/2)
# INIT_ORIENT = Quaternion(0.0, 0.0, 0.707107, 0.707107)
# INIT_ORIENT = Quaternion(0.0, 0.0, 0.0, 1.0)
INIT_OFFSET = 10.0

BACK_ORIENT = Quaternion(0.0, 0.0, 0.707107, 0.707107) # euler_to_quat(0, 0, math.pi/2)

STEP1 = "Step1. Capture the point cloud"
STEP2 = "Step2. Click grasping point"
STEP3 = "Step3. Move to the clicked point"
STEP4 = "Step4. Pick the object"
STEP5 = "Step5. Move to initial pose"

INIT_SOL = 1

PART_ATTACH_LINK = "left_inner_finger_pad"