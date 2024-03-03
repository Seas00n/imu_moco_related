import serial
import time
import numpy as np
from array import array
import sys
import rospy
# 设置正确的串口参数------------------------
ser_port = sys.argv[2]     #此处需要替换为对应使用的串口号，windows系统写成COMx，若是linux则要根据所用系统进行调整如写成/dev/ttyUSBx或/dev/ttySx
# ser_port = "/dev/ttyUSB0"
ser_baudrate = 115200 # 串口波特率

ser_timeout = 10 # specify your timeout here
# Open the serial port
ser = serial.Serial(ser_port, ser_baudrate, timeout=ser_timeout)

buffer_name = sys.argv[1]

if buffer_name == "right_thigh":
    mac_address = "d1:3d:df:93:34:a5"
    buffer_name = "imu_right_thigh"
elif buffer_name == "right_foot":
    mac_address = "7e:7f:3a:2d:bb:2a"
    buffer_name = "imu_right_foot"
elif buffer_name == "right_shank":
    mac_address = "6B:C3:BA:65:E3:86"
    buffer_name = "imu_right_shank"
elif buffer_name == "left_thigh":
    mac_address = "d8:72:db:51:4e:35"
    buffer_name = "imu_left_thigh"
elif buffer_name == "left_shank":
    mac_address = "b0:66:5b:88:d9:3a"
    buffer_name = "imu_left_shank"
elif buffer_name == "left_foot":
    mac_address = "17:2a:d2:d7:42:4f"
    buffer_name = "imu_left_foot"
elif buffer_name == "trunk":
    mac_address = "f5:ef:32:44:b3:1d"
    buffer_name = "imu_trunk"

imu_buffer = np.memmap("./log/{}.npy".format(buffer_name), dtype='float32', mode='r+',shape=(13,))
use_Acc = True

def Cmd_RxUnpack(buf, DLen):
    global imu_buffer, use_Acc
    scaleAccel       = 0.00478515625      # 加速度 [-16g~+16g]    9.8*16/32768
    scaleQuat        = 0.000030517578125  # 四元数 [-1~+1]         1/32768
    scaleAngle       = 0.0054931640625    # 角度   [-180~+180]     180/32768
    scaleAngleSpeed  = 0.06103515625      # 角速度 [-2000~+2000]    2000/32768
    scaleMag         = 0.15106201171875   # 磁场 [-4950~+4950]   4950/32768
    scaleTemperature = 0.01               # 温度
    scaleAirPressure = 0.0002384185791    # 气压 [-2000~+2000]    2000/8388608
    scaleHeight      = 0.0010728836       # 高度 [-9000~+9000]    9000/8388608

    imu_dat = array('f',[0.0 for i in range(0,34)])
    # print("rev data:",buf)
    if buf[0] == 0x11:
        ctl = (buf[2] << 8) | buf[1]
        print(buffer_name)
        print("\n subscribe tag: 0x%04x"%ctl)
        # print(" ms: ", ((buf[6]<<24) | (buf[5]<<16) | (buf[4]<<8) | (buf[3]<<0)))

        L =7 # 从第7字节开始根据 订阅标识tag来解析剩下的数据
        if ((ctl & 0x0001) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
            # print("\taX: %.3f"%tmpX); # x加速度aX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
            # print("\taY: %.3f"%tmpY); # y加速度aY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # print("\taZ: %.3f"%tmpZ); # z加速度aZ

            imu_dat[0] = float(tmpX)
            imu_dat[1] = float(tmpY)
            imu_dat[2] = float(tmpZ)
            if not use_Acc:
                imu_buffer[0] = imu_dat[0]
                imu_buffer[1] = imu_dat[1]
                imu_buffer[2] = imu_dat[2]
        
        if ((ctl & 0x0002) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # print("\tAX: %.3f"%tmpX) # x加速度AX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # print("\tAY: %.3f"%tmpY) # y加速度AY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # print("\tAZ: %.3f"%tmpZ) # z加速度AZ

            imu_dat[3] = float(tmpX)
            imu_dat[4] = float(tmpY)
            imu_dat[5] = float(tmpZ)
            if use_Acc:
                imu_buffer[0] = imu_dat[3]
                imu_buffer[1] = imu_dat[4]
                imu_buffer[2] = imu_dat[5]

        if ((ctl & 0x0004) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
            # print("\tGX: %.3f"%tmpX) # x角速度GX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
            # print("\tGY: %.3f"%tmpY) # y角速度GY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
            # print("\tGZ: %.3f"%tmpZ) # z角速度GZ

            imu_dat[6] = float(tmpX)
            imu_dat[7] = float(tmpY)
            imu_dat[8] = float(tmpZ)
            imu_buffer[3] = imu_dat[6]
            imu_buffer[4] = imu_dat[7]
            imu_buffer[5] = imu_dat[8]
        
        if ((ctl & 0x0008) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            # print("\tCX: %.3f"%tmpX); # x磁场CX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            # print("\tCY: %.3f"%tmpY); # y磁场CY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            # print("\tCZ: %.3f"%tmpZ); # z磁场CZ

            imu_dat[9] = float(tmpX)
            imu_dat[10] = float(tmpY)
            imu_dat[11] = float(tmpZ)
        
        if ((ctl & 0x0010) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleTemperature; L += 2
            # print("\ttemperature: %.2f"%tmpX) # 温度

            tmpU32 = np.uint32(((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L])))
            if ((tmpU32 & 0x800000) == 0x800000): # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                tmpU32 = (tmpU32 | 0xff000000)      
            tmpY = np.int32(tmpU32) * scaleAirPressure; L += 3
            # print("\tairPressure: %.3f"%tmpY); # 气压

            tmpU32 = np.uint32((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L]))
            if ((tmpU32 & 0x800000) == 0x800000): # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                tmpU32 = (tmpU32 | 0xff000000)
            tmpZ = np.int32(tmpU32) * scaleHeight; L += 3 
            # print("\theight: %.3f"%tmpZ); # 高度

            imu_dat[12] = float(tmpX)
            imu_dat[13] = float(tmpY)
            imu_dat[14] = float(tmpZ)

        if ((ctl & 0x0020) != 0):
            tmpAbs = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # print("\tw: %.3f"%tmpAbs); # w
            tmpX =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # print("\tx: %.3f"%tmpX); # x
            tmpY =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # print("\ty: %.3f"%tmpY); # y
            tmpZ =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # print("\tz: %.3f"%tmpZ); # z

            imu_dat[15] = float(tmpAbs)
            imu_dat[16] = float(tmpX)
            imu_dat[17] = float(tmpY)
            imu_dat[18] = float(tmpZ)
            imu_buffer[9] = imu_dat[16] # x
            imu_buffer[10] = imu_dat[17] # y
            imu_buffer[11] = imu_dat[18] # z
            imu_buffer[12] = imu_dat[15] # w

        if ((ctl & 0x0040) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            print("\tangleX: %.3f"%tmpX); # x角度
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            print("\tangleY: %.3f"%tmpY); # y角度
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            print("\tangleZ: %.3f"%tmpZ); # z角度

            imu_dat[19] = float(tmpX)
            imu_dat[20] = float(tmpY)
            imu_dat[21] = float(tmpZ)
            imu_buffer[6] = imu_dat[19]
            imu_buffer[7] = imu_dat[20]
            imu_buffer[8] = imu_dat[21]

        if ((ctl & 0x0080) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
            # print("\toffsetX: %.3f"%tmpX); # x坐标
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
            # print("\toffsetY: %.3f"%tmpY); # y坐标
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
            # print("\toffsetZ: %.3f"%tmpZ); # z坐标

            imu_dat[22] = float(tmpX)
            imu_dat[23] = float(tmpY)
            imu_dat[24] = float(tmpZ)

        if ((ctl & 0x0100) != 0):
            tmpU32 = ((buf[L+3]<<24) | (buf[L+2]<<16) | (buf[L+1]<<8) | (buf[L]<<0)); L += 4
            # print("\tsteps: %u"%tmpU32); # 计步数
            tmpU8 = buf[L]; L += 1
            if (tmpU8 & 0x01):# 是否在走路
                # print("\t walking yes")
                imu_dat[25] = 100
            else:
                # print("\t walking no")
                imu_dat[25] = 0
            if (tmpU8 & 0x02):# 是否在跑步
                # print("\t running yes")
                imu_dat[26] = 100
            else:
                # print("\t running no")
                imu_dat[26] = 0
            if (tmpU8 & 0x04):# 是否在骑车
                # print("\t biking yes")
                imu_dat[27] = 100
            else:
                # print("\t biking no")
                imu_dat[27] = 0
            if (tmpU8 & 0x08):# 是否在开车
                # print("\t driving yes")
                imu_dat[28] = 100
            else:
                # print("\t driving no")
                imu_dat[28] = 0

        if ((ctl & 0x0200) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # print("\tasX: %.3f"%tmpX); # x加速度asX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # print("\tasY: %.3f"%tmpY); # y加速度asY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # print("\tasZ: %.3f"%tmpZ); # z加速度asZ
        
            imu_dat[29] = float(tmpX)
            imu_dat[30] = float(tmpY)
            imu_dat[31] = float(tmpZ)
            
        if ((ctl & 0x0400) != 0):
            tmpU16 = ((buf[L+1]<<8) | (buf[L]<<0)); L += 2
            # print("\tadc: %u"%tmpU16); # adc测量到的电压值，单位为mv
            imu_dat[32] = float(tmpU16)

        if ((ctl & 0x0800) != 0):
            tmpU8 = buf[L]; L += 1
            # print("\t GPIO1  M:%X, N:%X"%((tmpU8>>4)&0x0f, (tmpU8)&0x0f))
            imu_dat[33] = float(tmpU8)
    else:
        print("[error] data head not define")

CmdPacket_Begin = 0x49   # 起始码
CmdPacket_End = 0x4D     # 结束码
CmdPacketMaxDatSizeRx = 73  # 模块发来的数据包的数据体最大长度

CS = 0  # 校验和
i = 0
RxIndex = 0

buf = bytearray(5 + CmdPacketMaxDatSizeRx) # 接收包缓存
cmdLen = 0 # 长度

def Cmd_GetPkt(byte):
    global CS, i, RxIndex, buf, cmdLen
    CS += byte # 边收数据边计算校验码，校验码为地址码开始(包含地址码)到校验码之前的数据的和
    if RxIndex == 0: # 起始码
        if byte == CmdPacket_Begin:
            i = 0
            buf[i] = CmdPacket_Begin
            i += 1
            CS = 0 # 下个字节开始计算校验码
            RxIndex = 1
    elif RxIndex == 1: # 数据体的地址码
        buf[i] = byte
        i += 1
        if byte == 255: # 255是广播地址，模块作为从机，它的地址不可会出现255
            RxIndex = 0
        else:
            RxIndex += 1
    elif RxIndex == 2: # 数据体的长度
        buf[i] = byte
        i += 1
        if byte > CmdPacketMaxDatSizeRx or byte == 0:  # 长度无效
            RxIndex = 0
        else:
            RxIndex += 1
            cmdLen = byte
    elif RxIndex == 3: # 获取数据体的数据
        buf[i] = byte
        i += 1
        if i >= cmdLen + 3: # 已收完数据体
            RxIndex += 1
    elif RxIndex == 4: # 对比 效验码
        CS -= byte
        if (CS&0xFF) == byte: # 校验正确
            buf[i] = byte
            i += 1
            RxIndex += 1
        else: # 校验失败
            RxIndex = 0
    elif RxIndex == 5: # 结束码
        RxIndex = 0
        if byte == CmdPacket_End: # 捕获到完整包
            buf[i] = byte
            i += 1
            hex_string = " ".join(f"{b:02X}" for b in buf[0:i])
            # print(f"U-Rx[Len={i}]:{hex_string}")
            Cmd_RxUnpack(buf[3:i-2], i-5) # 处理数据包的数据体
            t = time.time()
            print("Time:", t)
            return 1
    else:
        RxIndex = 0
    return 0

def check_sum(data):
    sum = 0
    for c in data:
        sum += int.from_bytes(bytes.fromhex(c),byteorder="big")
    sum = int(sum)
    return hex(sum&0xff).upper()[2:]

def cal_cmd(rate, data_type):
    rate = hex(rate).upper()[2:]
    cmd = "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF00FF49"
    data_struct = ["FF","0B","12","05","FF","00","04",rate,
            "01","03","05",data_type[2:],data_type[0:2]]
    sum = check_sum(data_struct)
    for i in data_struct:
        cmd = cmd+i
    cmd  = cmd+sum+"4D"
    return cmd


def read_data():
    cmd = cal_cmd(rate=int(50),data_type="0067")# data_type必须为4位
    ser.write( bytes.fromhex(cmd) ) #1.发送配置参数
    time.sleep(0.2)
    ser.write( bytes.fromhex("0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF00FF49FF0103034D") ) #2.唤醒传感器
    time.sleep(0.2)
    ser.write( bytes.fromhex("0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF00FF49FF0119194D") ) #3.开启主动上报
    while True:
        data = ser.read(1) # read 1 bytes
        if len(data) > 0: # if data is not empty
            Cmd_GetPkt(data[0])

# Start reading data
read_data()

