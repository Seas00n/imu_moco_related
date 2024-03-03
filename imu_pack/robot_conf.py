import os
import numpy as np
from scipy.spatial.transform import Rotation as R
np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60

mass = 67
height = 1.78
foot_length = 0.24
shank_length = 0.4
thigh_length = 0.4
trunk_length = 0.7


right_stance_path = "./urdf/"
right_stance_urdf = right_stance_path+"rt_model_fk.urdf"

left_stance_path = "./urdf/"
left_stance_urdf = left_stance_path+"lt_model_fk.urdf"

center_path = "./urdf/"
center_urdf = center_path+"ce_model_fk.urdf"

rf_joint_name = "ankle_right"
lf_joint_name = "ankle_left"
rf_frame_name = "foot_right"
lf_frame_name = "foot_left"
trunk_frame_name = "trunk_inertia_fixed"
rf_toe_frame_name = "foot_right_contact0"
lf_toe_frame_name = "foot_left_contact0"

# q_raw = [0:q_rt, 1:q_rs, 2:q_rf, 3:q_lt, 4:q_ls, 5:q_lf, 6:q_tr]
# q_rf = [q_rtoe, q_ra, q_rk, q_rh, q_lh, q_lk, q_la]
# q_lf = [q_ltoe, q_la, q_lk, q_lh, q_rh, q_rk, q_ra]
# q_ce_without_trunk = [q_rh, q_rk, q_ra, q_lh, q_lk, q_la]
# q_ce = [p_trunk, quat_trunk, q_rh, q_rk, q_ra, q_lh, q_lk, q_la]
# q_dataset = [q_rh_ds, q_rk_ds, q_ra_ds, q_lh_ds, q_lk_ds, q_la_ds]

def switch_map_rf_lf(q_now):
    q_next = np.zeros_like(q_now)
    q_next[0] = np.sum(q_now)
    q_next[1:] = -np.flip(q_now[1:])
    return q_next

def switch_map_rf_ce_without_trunk(q_now):
    q = np.zeros((6,))
    q[0:3] = q_now[4:7]
    q[3:] =  -np.flip(q_now[1:4])
    return q

def switch_map_lf_ce_without_trunk(q_now):
    q = np.zeros((6,))
    q[0:3] = -np.flip(q_now[1:4])
    q[3:] = q_now[4:7]
    return q

def switch_map_rf_ce(q_now, p_trunk, R_trunk):
    q = np.zeros(np.shape(q_now)[0]+6)
    q[7:10] = q_now[4:7]
    q[10:] = -np.flip(q_now[1:4])
    q[0:3] = p_trunk
    q[3:7] =  R.from_matrix(R_trunk).as_quat()
    return q

def switch_map_lf_ce(q_now, p_trunk, R_trunk):
    q = np.zeros(np.shape(q_now)[0]+6)
    q[7:10] = -np.flip(q_now[1:4])
    q[10:] = q_now[4:7]
    q[0:3] = p_trunk
    q[3:7] =  R.from_matrix(R_trunk).as_quat()
    return q


def switch_q_raw_rf(q_raw):
    q = np.zeros((7,))
    q[0] = -q_raw[2]
    q[1] = q_raw[1]+q_raw[2]
    q[2] = q_raw[0]-q_raw[1]
    q[3] = q_raw[6]-q_raw[0]
    q[4] = q_raw[3]-q_raw[6]
    q[5] = q_raw[4]-q_raw[3]
    q[6] = -q_raw[5]-q_raw[4]
    return q

def switch_q_raw_lf(q_raw):
    q = np.zeros((7,))
    q[0] = -q_raw[5]
    q[1] = q_raw[4]+q_raw[5]
    q[2] = q_raw[3]-q_raw[4]
    q[3] = q_raw[6]-q_raw[3]
    q[4] = q_raw[0]-q_raw[6]
    q[5] = q_raw[1]-q_raw[0]
    q[6] = -q_raw[2]-q_raw[1]
    return q

def switch_q_raw_ce_without_trunk(q_raw):
    q = np.zeros((6,))
    q[0] = q_raw[0]
    q[1] = q_raw[1]-q_raw[0]
    q[2] = -q_raw[2]-q_raw[1]
    q[3] = q_raw[3]
    q[4] = q_raw[4]-q_raw[3]
    q[5] = -q_raw[5]-q_raw[4]
    return q

def switch_q_raw_ce(q_raw, p_trunk, R_trunk):
    q = np.zeros(6+7)
    q[0:3] = p_trunk
    q[3:7] = R.from_matrix(R_trunk).as_quat()
    q[7] = q_raw[0]
    q[8] = q_raw[1]-q_raw[0]
    q[9] = -q_raw[2]-q_raw[1]
    q[10] = q_raw[3]
    q[11] = q_raw[4]-q_raw[3]
    q[12] = -q_raw[5]-q_raw[4]
    return q

def switch_q_raw_dataset(q_raw):
    q = np.zeros((6,))
    q[0] = -q_raw[0]
    q[1] = q_raw[1]-q_raw[0]
    q[2] = q_raw[2]+q_raw[1]
    q[3] = -q_raw[3]
    q[4] = q_raw[4]-q_raw[3]
    q[5] = q_raw[5]+q_raw[4]
    return q


def robot_q_diff(q_ce_mat, dt):
    nq = np.shape(q_ce_mat)[1]
    dq_vec = np.zeros((nq-1,))
    ddq_vec = np.zeros((nq-1,))
    eular0 = R.from_quat(q_ce_mat[-3,3:7]).as_euler('xyz',degrees=False)
    eular1 = R.from_quat(q_ce_mat[-2,3:7]).as_euler('xyz',degrees=False)
    eular2 = R.from_quat(q_ce_mat[-1,3:7]).as_euler('xyz',degrees=False)
    dq_vec[0:3] = (q_ce_mat[-1,0:3]-q_ce_mat[-2,0:3])/dt
    dq_vec[3:6] = (eular2-eular1)/dt
    dq_vec[6:] = (q_ce_mat[-1,7:]-q_ce_mat[-2,7:])/dt
    ddq_vec[0:3] = (q_ce_mat[-1,0:3]+q_ce_mat[-3,0:3]-2*q_ce_mat[-2,0:3])/dt/dt
    ddq_vec[3:6] = (eular2+eular0-2*eular1)/dt/dt
    ddq_vec[6:] = (q_ce_mat[-1,7:]+q_ce_mat[-3,7:]-2*q_ce_mat[-2,7:])/dt/dt
    return dq_vec, ddq_vec



def diff_quat(q1,q0,dt):
    eular0 = R.from_quat(q0).as_euler('xyz',degrees=False)
    eular1 = R.from_quat(q1).as_euler('xyz',degrees=False)
    return np.array([0, (eular1[1]-eular0[1])/dt, 0])

q_rf_pose1 = [0, 0.3, -0.7, 0.8, -0.34, 1.06, -0.2]
q_rf_walk_start = [0, -0.1, -0.2, 0.3, 0.3, 0.65, 0]
q_rf_walk_mid = [0, 0, 0 , 0, -0.3, 1.2, -0.2]
q_rf_walk_end = [0.95, 0, -0.65, -0.3, -0.3, 0.2, 0.1]
q_lf_walk_start = switch_map_rf_lf(q_rf_walk_end)
q_lf_walk_mid = [0, 0, 0 , 0, 0.3, 1.2, -0.2]
q_lf_walk_end = [0.95, 0, -0.65, -0.3, -0.3, 0.2, 0.1]


