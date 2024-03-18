import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import datetime
import scipy.io as scio

path_log = "./log/"

imu_rf_bf = np.memmap(path_log+"imu_right_foot.npy", dtype='float32', mode='r',shape=(13,))

q_init = np.zeros((7,))

save_path = "data_record1.npy"

def init_q(init_counter):
    global imu_rf_bf
    global q_init
    q_init = np.zeros((7,))
    for i in range(init_counter):
        q_init[:] += np.array([0, 0,imu_rf_bf[6], 0, 0, 0, 0]).reshape((7,))
        time.sleep(0.01)
        cv2.waitKey(1)
    q_init[:] = q_init/init_counter

if __name__ == "__main__":
    img = np.zeros((300, 300), np.uint8)
    img.fill(200)
    cv2.imshow("Press s to start, z to save, q to exit",img)

    print("初始化关节角度")    
    init_q(50)
    print("关节角度初始化完毕")
    print("在图框按s开始记录，按z存储，按q退出")

    data_save_buffer = []

    begin_save = False

    t0 = datetime.datetime.now()
    while True:
        data_rf = np.copy(imu_rf_bf[:])
        q_rf = (imu_rf_bf[6]-q_init[2])*np.pi/180
        qd_rf = imu_rf_bf[3:6]
        qdd_rf = imu_rf_bf[0:3]
        print("\r q_ankle = "+format(q_rf, ">6.2f")+
              ", qd_ankle = "+format(np.linalg.norm(qd_rf), ">6.2f")+
              ", qdd_ankle = "+format(np.linalg.norm(qdd_rf), ">6.2f"), end='')
        if begin_save:
            t_now = datetime.datetime.now()
            data_save_buffer.append(
                np.append(data_rf, (t_now-t0).seconds)
            )
            print("Saving")
        cmd = cv2.waitKey(1)
        if cmd == ord('s') and not begin_save:
            begin_save = True
            t0 = datetime.datetime.now()
            print("Begin to Save")
        elif cmd == ord('z') and begin_save:
            begin_save = False
            
            np.save(save_path, np.array(data_save_buffer))
            data_save_buffer = []
            print("Save At "+save_path)
        elif cmd == ord('q'):
            print("Finish")
            cv2.destroyAllWindows()
            break
        time.sleep(0.01)
    
    fig = plt.figure(figsize=(10,5))
    ax = fig.add_subplot(111)
    data_vec = np.load(save_path)
    idx = np.arange(np.shape(data_vec)[0])
    ax.plot(idx, data_vec[:, 6])
    
    plt.show()
    plt.close()



        