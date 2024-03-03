import numpy as np
import sys
import lcm
import time
import cv2
import rospy
import matplotlib.pyplot as plt
import datetime
sys.path.append('/home/yuxuan/Project/HPS_Perception/map_ws/src/HPS_Perception/hps_moco/scripts')



path_log = "./log/"

imu_rt_bf = np.memmap(path_log+"imu_right_thigh.npy", dtype='float32', mode='r',shape=(13,))
imu_rs_bf = np.memmap(path_log+"imu_right_shank.npy", dtype='float32', mode='r',shape=(13,))
imu_rf_bf = np.memmap(path_log+"imu_right_foot.npy", dtype='float32', mode='r',shape=(13,))
imu_lt_bf = np.memmap(path_log+"imu_left_thigh.npy", dtype='float32', mode='r',shape=(13,))
imu_ls_bf = np.memmap(path_log+"imu_left_shank.npy", dtype='float32', mode='r',shape=(13,))
imu_lf_bf = np.memmap(path_log+"imu_left_foot.npy", dtype='float32', mode='r',shape=(13,))
imu_tr_bf = np.memmap(path_log+"imu_trunk.npy", dtype='float32', mode='r',shape=(13,))

img_path = "/home/yuxuan/Project/HPS_Perception/map_ws/src/HPS_Perception/hps_moco/img/"
img_pause = cv2.imread(img_path+"Pause.png",cv2.IMREAD_COLOR)
img_walk = cv2.imread(img_path+"Walk.png",cv2.IMREAD_COLOR)
img_init = cv2.imread(img_path+"Init.png",cv2.IMREAD_COLOR)

q_init = np.zeros((7,))

def init_q(init_counter):
    global imu_rt_bf, imu_rs_bf, imu_rf_bf, imu_lt_bf, imu_ls_bf, imu_lf_bf, imu_tr_bf
    global q_init
    q_init = np.zeros((7,))
    for i in range(init_counter):
        q_init[:] += np.array([imu_rt_bf[6], imu_rs_bf[6],
                              imu_rf_bf[6], imu_lt_bf[6],
                              imu_ls_bf[6], imu_lf_bf[6], imu_tr_bf[6]]).reshape((7,))
        time.sleep(0.01)
        cv2.waitKey(1)
    q_init[:] = q_init/init_counter

save_path = "data_record_7.npy"

if __name__ == "__main__":
    rospy.init_node("walkerPub", anonymous=True)
    rate = rospy.Rate(80)

    gait_recorder = []

    start_batch_opt = False
    
    cv2.imshow("Press s, c, z, q",img_pause)

    init_q(init_counter=50)

    L_in_stance = True
    R_in_stance = True

    data_sim_save = []
    t0 = datetime.datetime.now()
    while not rospy.is_shutdown():
        try:
            # 操作台
            cmd = cv2.waitKey(1)
            if not start_batch_opt and cmd == ord('s'):# 按s开始记录
                data_sim_save = []
                start_batch_opt = True
                t0 = datetime.datetime.now()
                cv2.imshow("Press s, c, z, q",img_walk)
            elif start_batch_opt and cmd == ord('c'):# 按c存储数据
                start_batch_opt = False
                cv2.imshow("Press s, c, z, q",img_pause)
                np.save(save_path, np.array(data_sim_save))
                print("Result Has Been Saved")
            elif not start_batch_opt and cmd == ord('z'):# 按z重新初始化角度
                cv2.imshow("Press s, c, z, q",img_init)
                init_q(50)
                cv2.imshow("Press s, c, z, q",img_pause)
                continue
            elif cmd == ord('q'):# 按q退出并绘制
                break
             
            q_rt = (imu_rt_bf[6] - q_init[0])*np.pi/180
            q_rs = (imu_rs_bf[6] - q_init[1])*np.pi/180
            q_rf = (imu_rf_bf[6] - q_init[2])*np.pi/180
            q_lt = (imu_lt_bf[6] - q_init[3])*np.pi/180
            q_ls = (imu_ls_bf[6] - q_init[4])*np.pi/180
            q_lf = (imu_lf_bf[6] - q_init[5])*np.pi/180
            q_tr = (imu_tr_bf[6] - q_init[6])*np.pi/180
            qdd_rt = imu_rt_bf[0:3]
            qdd_lt = imu_lt_bf[0:3]
            qd_rf = imu_rf_bf[3:6]*np.pi/180
            qd_lf = imu_lf_bf[3:6]*np.pi/180

            if start_batch_opt:
                t = datetime.datetime.now()
                # TODO: 将需要采集的imu数据写在这里
                data_sim_save.append([
                    q_rt, q_rs, q_rf, q_lt, q_ls, q_lf, q_tr, 
                    qd_rf[0], qd_rf[1], qd_rf[2],
                    qd_lf[0], qd_lf[1], qd_lf[2],
                    qdd_rt[0], qdd_rt[1], qdd_rt[2], 
                    qdd_lt[0], qdd_lt[1], qdd_lt[2],(t-t0).seconds
                ])
                np.set_printoptions(precision=2)
                print(np.array(data_sim_save[-1]))
            rate.sleep()
        except Exception as e:
            print(e)
            break

    data_all = np.load(save_path)
    # 存储的imu角度和数据集的朝向有差异，需要进行解算得到关节角度
    '''
    q_dataset = [q_lt,
        q_ls-q_lt,
        -q_lf-q_ls,
        q_rt,
        q_rs-q_rt,
        -q_rf-q_rs] 
    '''
    q_rt_all = -data_all[:,0]
    q_rs_all = data_all[:,1]-data_all[:,0]
    q_rf_all = data_all[:,2]+data_all[:,1]
    q_lt_all = -data_all[:,3]
    q_ls_all = data_all[:,4]-data_all[:,3]
    q_lf_all = data_all[:,5]+data_all[:,4]
    idx = np.arange(np.shape(data_all)[0])
    fig = plt.figure(figsize=(10,5))
    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)
    ax1.plot(idx, q_lt_all)
    ax1.plot(idx, q_ls_all)
    ax1.plot(idx, q_lf_all)
    ax2.plot(idx, q_rt_all)
    ax2.plot(idx, q_rs_all)
    ax2.plot(idx, q_rf_all)
    plt.show()