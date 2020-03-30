import tf
from utils import *

def convert_ref_frame(wpt):
    '''
    in : [position, orien(euler)]
    convert tf from /base_footprint -> /real_ee_link : p1
                to /real_base_link -> /real_ee_link  : p0
                p0 = T01*p1
    '''
    # start_pose = convert_ref_frame(cvrg_waypoints[0])
    # trans, rot = pose_to_tr(start_pose)
    # trans, rot = convert_ref_frame(cvrg_waypoints[0])
    pose_wrt_base = tr_to_mat(wpt[:3], wpt[3:])
    t_ur5_base = tr_to_mat([-0.000, -0.175, -0.675], [-0.000, -0.000, 0.707, 0.707])
    pose_wrt_ur5 = mat_to_pose(tf.transformations.concatenate_matrices(t_ur5_base, pose_wrt_base))
    return pose_wrt_ur5

converted_pose = convert_ref_frame([0.347176858326, 0.468640011366, 1.14603542809, 0.765530197885, -0.643049350175, 0.0140599077307, 0.015917548841])
print converted_pose