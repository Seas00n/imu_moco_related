import numpy as np
import robot_conf as conf
from plot_utils import FastPlot2DHuman
import tsid
import pinocchio as pin
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator

data_path = "./data/"
q_dequeue = np.load(data_path+"q.npy")
#idx_HS = [左脚第一次heel_strike的idx=0, 左脚第二次heel_strike的idx,
#          右脚第一次heel_strike的idx, 右脚第二次heel_strike的idx=np.shape(q_dequeue)[0]]
idx_HS = np.load(data_path+"idx_HS.npy")
print(idx_HS)


robot_ce = tsid.RobotWrapper(conf.center_urdf,
                          [conf.center_path], 
                          pin.JointModelFreeFlyer(), False)
model_ce = robot_ce.model()
data_ce = robot_ce.data()

robot_rf = tsid.RobotWrapper(conf.right_stance_urdf,
                          [conf.right_stance_path], False)
model_rf = robot_rf.model()
data_rf = robot_rf.data()

robot_lf = tsid.RobotWrapper(conf.left_stance_urdf,
                             [conf.left_stance_urdf],False)
model_lf = robot_lf.model()
data_lf = robot_lf.data()

#左脚支撑,右脚heel_strike此时仍然可以使用左脚支撑的urdf模型计算运动学
q_raw0 = q_dequeue[idx_HS[2]] 
q_lf0 = conf.switch_q_raw_lf(q_raw0)
#利用robot_lf计算躯干的相对位移动
robot_lf.computeAllTerms(data_lf, q_lf0, np.zeros(robot_lf.nv))
p_trunk_lf0 = robot_lf.framePosition(data_lf, 
                                     model_lf.getFrameId(conf.trunk_frame_name)).translation
R_trunk_lf0 = robot_lf.framePosition(data_lf, 
                                     model_lf.getFrameId(conf.trunk_frame_name)).rotation
#利用robot_ce绘制运动学
q_ce0 = conf.switch_map_lf_ce(q_lf0,
                              p_trunk=p_trunk_lf0,
                              R_trunk=R_trunk_lf0)
fig = plt.figure()
ax = plt.subplot(111)
ax.set_xlim(-1,2)
ax.set_ylim(-0.2,2.2)
ax.set_title('Forward Kinematics', 
                   fontsize=10,
                   fontdict={"weight":'bold'})
ax.set_xlabel('x/m',fontsize=12,
                   fontdict={"weight":'bold'})
ax.set_ylabel('z/m',fontsize=12,
                   fontdict={"weight":'bold'})
ax.xaxis.set_major_locator(MultipleLocator(1))
ax.yaxis.set_major_locator(MultipleLocator(1))
viewer1 = FastPlot2DHuman(robot=robot_ce, ax=ax)
pin.forwardKinematics(model_ce, data_ce, q=q_ce0)
viewer1.set_skeleton_stance(robot_ce, data_ce)


#右支撑,左脚第二次heel_strike此时仍然可以使用右脚支撑的urdf模型计算运动学
q_raw1 = q_dequeue[idx_HS[1]]
q_rf1 = conf.switch_q_raw_rf(q_raw1)
#利用robot_lf计算躯干的相对位移动
robot_rf.computeAllTerms(data_rf, q_rf1, np.zeros(robot_rf.nv))
p_trunk_rf1 = robot_rf.framePosition(data_rf, 
                                     model_rf.getFrameId(conf.trunk_frame_name)).translation+[0.6,0,0]
R_trunk_rf1 = robot_rf.framePosition(data_rf, 
                                     model_rf.getFrameId(conf.trunk_frame_name)).rotation
#利用robot_ce绘制运动学
q_ce1 = conf.switch_map_rf_ce(q_rf1,
                              p_trunk=p_trunk_rf1,
                              R_trunk=R_trunk_rf1)

viewer2 = FastPlot2DHuman(robot=robot_ce, ax=ax)
pin.forwardKinematics(model_ce, data_ce, q=q_ce1)
viewer2.set_skeleton_stance(robot_ce, data_ce)

plt.show(block=True)
