import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import robot_conf as conf
from scipy.spatial.transform import Rotation as R
class FastPlot2DHuman:
    ax: matplotlib.axes._axes.Axes
    fig: matplotlib.figure.Figure
    def __init__(self, robot, is_right=True, ax=None):
        plt.show(block=False)
        self.assign_robot(robot=robot, is_right=is_right)
        if ax is None:
            self.fig, self.ax = plt.subplots()
            self.fig.canvas.draw()
        else:
            self.ax = ax
        self.element_init(self.ax)

    def element_init(self, ax):
        self.toe_right = self.ax.scatter(np.zeros(1,),np.zeros(1,),color='b')
        self.toe_left = self.ax.scatter(np.zeros(1,),np.zeros(1,),color='b')
        self.ankle_right = self.ax.scatter(np.zeros(1,),np.zeros(1,),color='g')
        self.ankle_left = self.ax.scatter(np.zeros(1,),np.zeros(1,),color='g')
        self.knee_right =  self.ax.scatter(np.zeros(1,),np.zeros(1,),color='y')
        self.knee_left =  self.ax.scatter(np.zeros(1,),np.zeros(1,),color='y')
        self.hip_right = self.ax.scatter(np.zeros(1,),np.zeros(1,),color='r')
        self.hip_left = self.ax.scatter(np.zeros(1,),np.zeros(1,),color='r')
        self.trunk_center = self.ax.scatter(np.zeros(1,),np.zeros(1,),color='m',linewidths=2)
        self.trunk_patch = patches.Rectangle((0,0),0.3,0.7,edgecolor='black',fill=False,linewidth=2)
        self.line_foot_right = self.ax.plot(np.zeros(1,),linewidth=2,color='red')[0]
        self.line_shank_right = self.ax.plot(np.zeros(1,),linewidth=2, color='red')[0]
        self.line_thigh_right = self.ax.plot(np.zeros(1,),linewidth=2, color='red')[0]
        self.line_foot_left = self.ax.plot(np.zeros(1,),linewidth=2,color='blue')[0]
        self.line_shank_left = self.ax.plot(np.zeros(1,),linewidth=2, color='blue')[0]
        self.line_thigh_left = self.ax.plot(np.zeros(1,),linewidth=2, color='blue')[0]
        self.ax.add_patch(self.trunk_patch)
        self.state_text = self.ax.text(-0.4,1.7,"",fontsize=10,fontdict={"weight":'bold'})
        self.state_text.set_color("#5B6060")

    def assign_robot(self,robot,is_right=True):
        self.model = robot.model()
        self.id_toe_right = self.model.getFrameId("foot_right_contact0")
        self.id_ankle_right = self.model.getFrameId("foot_right_contact2")
        # self.id_toe_right = self.model.getFrameId("ankle_right")
        # self.id_ankle_right = self.model.getFrameId("ankle_right")
        self.id_knee_right = self.model.getFrameId("knee_right")
        self.id_hip_right = self.model.getFrameId("hip_right")
        self.id_hip_left = self.model.getFrameId("hip_left")
        self.id_knee_left = self.model.getFrameId("knee_left")
        self.id_toe_left = self.model.getFrameId("foot_left_contact0")
        self.id_ankle_left = self.model.getFrameId("foot_left_contact2")
        # self.id_toe_left = self.model.getFrameId("ankle_left")
        # self.id_ankle_left = self.model.getFrameId("ankle_left")
        self.id_trunk_center = self.model.getFrameId(conf.trunk_frame_name)

    def update_canvas(self):
        self.ax.set_xlim(-1,2)
        self.ax.set_ylim(-0.2,2.2)
        self.ax.set_aspect('equal', adjustable='box')
        # self.fig.canvas.update()
        self.fig.canvas.flush_events()
        self.ax.draw_artist(self.ax.patch)

    def set_skeleton_stance(self, robot, data, base_shift=np.array([0,0])):
        pos_toe_right = robot.framePosition(data, self.id_toe_right).translation[0::2]+base_shift
        self.toe_right.set_offsets(pos_toe_right)
        pos_ankle_right = robot.framePosition(data, self.id_ankle_right).translation[0::2]+base_shift
        self.ankle_right.set_offsets(pos_ankle_right)
        self.line_foot_right.set_xdata([pos_toe_right[0], pos_ankle_right[0]])
        self.line_foot_right.set_ydata([pos_toe_right[1], pos_ankle_right[1]])
        pos_knee_right = robot.framePosition(data, self.id_knee_right).translation[0::2]+base_shift
        self.knee_right.set_offsets(pos_knee_right)
        self.line_shank_right.set_xdata([pos_ankle_right[0],pos_knee_right[0]])
        self.line_shank_right.set_ydata([pos_ankle_right[1],pos_knee_right[1]])
        pos_hip_right = robot.framePosition(data, self.id_hip_right).translation[0::2]+base_shift
        self.hip_right.set_offsets(pos_hip_right)
        self.line_thigh_right.set_xdata([pos_knee_right[0],pos_hip_right[0]])
        self.line_thigh_right.set_ydata([pos_knee_right[1],pos_hip_right[1]])
        pos_toe_left = robot.framePosition(data, self.id_toe_left).translation[0::2]+base_shift
        self.toe_left.set_offsets(pos_toe_left)
        pos_ankle_left = robot.framePosition(data, self.id_ankle_left).translation[0::2]+base_shift
        self.ankle_left.set_offsets(pos_ankle_left)
        self.line_foot_left.set_xdata([pos_toe_left[0], pos_ankle_left[0]])
        self.line_foot_left.set_ydata([pos_toe_left[1], pos_ankle_left[1]])
        pos_knee_left = robot.framePosition(data, self.id_knee_left).translation[0::2]+base_shift
        self.knee_left.set_offsets(pos_knee_left)
        self.line_shank_left.set_xdata([pos_ankle_left[0],pos_knee_left[0]])
        self.line_shank_left.set_ydata([pos_ankle_left[1],pos_knee_left[1]])
        pos_hip_left = robot.framePosition(data, self.id_hip_left).translation[0::2]+base_shift
        self.hip_left.set_offsets(pos_hip_left)
        self.line_thigh_left.set_xdata([pos_knee_left[0],pos_hip_left[0]])
        self.line_thigh_left.set_ydata([pos_knee_left[1],pos_hip_left[1]])
        self.set_trunk(robot=robot,data=data,base_shift=base_shift)
        
    def set_trunk(self, robot, data,base_shift=np.array([0,0])):
        pose_trunk = robot.framePosition(data, self.id_trunk_center)
        self.trunk_center.set_offsets(pose_trunk.translation[0::2]+base_shift)
        R_trunk = pose_trunk.rotation
        # p_patch = np.array([-0.1, 0, -0.26]).reshape((3,1))
        p_patch = np.array([-0.15, 0, -0.35]).reshape((3,1))
        p_patch = (R_trunk@p_patch).reshape((3,))[0::2]+pose_trunk.translation[0::2]+base_shift
        tf1 = matplotlib.transforms.Affine2D().translate(p_patch[0],p_patch[1])
        deg = R.from_matrix(R_trunk).as_euler('xyz',degrees=True)
        tf2 = matplotlib.transforms.Affine2D().rotate_deg(-deg[1])
        self.trunk_patch.set_transform(tf2+tf1+self.ax.transData)
        
def fifo_vec(data_vec, data):
    data_vec[0:-1] = data_vec[1:]
    data_vec[-1] = data
    return data_vec

class FastPlotCurve:
    ax: matplotlib.axes._axes.Axes
    fig: matplotlib.figure.Figure
    def __init__(self):
        self.fig, axes= plt.subplots(nrows=2,ncols=1)
        self.axr = axes[0]
        self.axl = axes[1]
        self.fig.canvas.draw()
        self.xdata = np.arange(0,100)
        self.yRT = np.zeros_like(self.xdata)
        self.yRS = np.zeros_like(self.xdata)
        self.yRA = np.zeros_like(self.xdata)
        self.line_RT = self.axr.plot(self.xdata,self.yRT, linewidth=2, color='red')[0]
        self.line_RS = self.axr.plot(self.xdata,self.yRS, linewidth=2, color='blue')[0]
        self.line_RA = self.axr.plot(self.xdata,self.yRA, linewidth=2, color='green')[0]
        self.yLT = np.zeros_like(self.xdata)
        self.yLS = np.zeros_like(self.xdata)
        self.yLA = np.zeros_like(self.xdata)
        self.line_LT = self.axl.plot(self.xdata,self.yLT, linewidth=2, color='red')[0]
        self.line_LS = self.axl.plot(self.xdata,self.yLS, linewidth=2, color='blue')[0]
        self.line_LA = self.axl.plot(self.xdata,self.yLA, linewidth=2, color='green')[0]
        self.divide_x = 99
        self.divide_r = self.axr.plot(self.divide_x*np.ones((2,)),np.linspace(-90,90,2),'--m')[0]
        self.divide_l = self.axl.plot(self.divide_x*np.ones((2,)),np.linspace(-90,90,2),'--m')[0]
    def set_data(self,q,gait_over_flag):
        self.yRT = fifo_vec(self.yRT, q[0])
        self.yRS = fifo_vec(self.yRS, q[1])
        self.yRA = fifo_vec(self.yRA, q[2])
        self.yLT = fifo_vec(self.yLT, q[3])
        self.yLS = fifo_vec(self.yLS, q[4])
        self.yLA = fifo_vec(self.yLA, q[-1])
        self.line_RT.set_ydata(self.yRT)
        self.line_RS.set_ydata(self.yRS)
        self.line_RA.set_ydata(self.yRA)
        self.line_LT.set_ydata(self.yLT)
        self.line_LS.set_ydata(self.yLS)
        self.line_LA.set_ydata(self.yLA)
        if gait_over_flag:
            self.divide_x = 99
        else:
            self.divide_x -= 1
            if self.divide_x < 0:
                self.divide_x = 0
        self.divide_r.set_xdata(np.ones((2,))*self.divide_x)
        self.divide_l.set_xdata(np.ones((2,))*self.divide_x)

    def update_canvas(self):
        self.fig.canvas.flush_events()
        self.axr.draw_artist(self.axr.patch)
        self.axl.draw_artist(self.axl.patch)

