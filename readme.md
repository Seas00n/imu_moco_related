# imu蓝牙mac_address修改
在imu_blue.py中修改每个imu的mac_address

mac_address可以在上位机软件查询
# imu_buffer内容
```
imu.linear_acceleration.x = imu_data[0]
imu.linear_acceleration.y = imu_data[1]
imu.linear_acceleration.z = imu_data[2]
imu.angular_velocity.x = imu_data[3]*np.pi/180
imu.angular_velocity.y = imu_data[4]*np.pi/180
imu.angular_velocity.z = imu_data[5]*np.pi/180
imu.eular.roll = imu_data[6]*np.pi/180
imu.eular.pitch = imu_data[7]*np.pi/180
imu.eular.yaw = imu_data[8]*np.pi/180
imu.orientation.x = imu_data[9]
imu.orientation.y = imu_data[10]
imu.orientation.z = imu_data[11]
imu.orientation.w = imu_data[12]
```

# memmap接收
```
imu_rt_bf = np.memmap(path_log+"imu_right_thigh.npy", dtype='float32', mode='r',shape=(13,))
imu_rs_bf = np.memmap(path_log+"imu_right_shank.npy", dtype='float32', mode='r',shape=(13,))
imu_rf_bf = np.memmap(path_log+"imu_right_foot.npy", dtype='float32', mode='r',shape=(13,))
imu_lt_bf = np.memmap(path_log+"imu_left_thigh.npy", dtype='float32', mode='r',shape=(13,))
imu_ls_bf = np.memmap(path_log+"imu_left_shank.npy", dtype='float32', mode='r',shape=(13,))
imu_lf_bf = np.memmap(path_log+"imu_left_foot.npy", dtype='float32', mode='r',shape=(13,))
imu_tr_bf = np.memmap(path_log+"imu_trunk.npy", dtype='float32', mode='r',shape=(13,))
```

# 数据采集
确保蓝牙可以稳定连接4个imu，不然就换串口
```
roslaunch hps_moco imu_blue.launch
roslaunch hps_moco imu_serial.launch
python3 online_imu_record.py
```
键盘操作开始录制或者初始化等

# 运动学分析
```
python3 forward_kinematics.py
```

# urdf尺寸修改
在conf中修改相关尺寸
```
mass = 66
height = 1.78
foot_length = 0.24
shank_length = 0.4
thigh_length = 0.4
trunk_length = 0.7
```
```
python3 change_xacro.py
sh xacro_to_urdf.sh
```