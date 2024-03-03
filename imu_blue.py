from ast import Try
from time import sleep
import numpy as np
from numpy import array
import gatt
import time
import rospy
from argparse import ArgumentParser
from array import array
import socket
import sys

buffer_name = sys.argv[1]
color_head = "\033[38m"
color_tail = "\033[0m"

if buffer_name == "right_thigh":
    mac_address = "d1:3d:df:93:34:a5"
    buffer_name = "imu_right_thigh"
elif buffer_name == "right_foot":
    mac_address = "7e:7f:3a:2d:bb:2a"
    buffer_name = "imu_right_foot"
    color_head = "\033[38m"
elif buffer_name == "right_shank":
    mac_address = "6B:C3:BA:65:E3:86"
    buffer_name = "imu_right_shank"
    color_head = "\033[32m"
elif buffer_name == "left_thigh":
    mac_address = "d8:72:db:51:4e:35"
    buffer_name = "imu_left_thigh"
elif buffer_name == "left_shank":
    mac_address = "b0:66:5b:88:d9:3a"
    buffer_name = "imu_left_shank"
    color_head = "\033[33m"
elif buffer_name == "left_foot":
    mac_address = "17:2a:d2:d7:42:4f"
    buffer_name = "imu_left_foot"
    color_head = "\033[34m"
elif buffer_name == "trunk":
    mac_address = "f5:ef:32:44:b3:1d"
    buffer_name = "imu_trunk"

use_Acc = True


class AnyDevice(gatt.Device):
    sock_pc = None
    parse_imu_flage = False
    imu_buffer = np.memmap("./log/{}.npy".format(buffer_name), dtype='float32', mode='r+',
                           shape=(13,))
    def connect_succeeded(self):
        super().connect_succeeded()
        #print("[%s] Connected" % (self.mac_address))

    def connect_failed(self, error):
        super().connect_failed(error)
        #print("[%s] Connection failed: %s" % (self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        #print("[%s] Disconnected" % (self.mac_address))

    def services_resolved(self):
        super().services_resolved()

        #print("[%s] Resolved services" % (self.mac_address))
        for service in self.services:
            #print("[%s]\tService [%s]" % (self.mac_address, service.uuid))
            for characteristic in service.characteristics:
                print("[%s]\t\tCharacteristic [%s]" % (self.mac_address, characteristic.uuid))

        # 保持连接
        lzchar1 = next(
            c for c in service.characteristics
            if c.uuid == '0000ae01-0000-1000-8000-00805f9b34fb'.lower())
        lzchar1.write_value(')'.encode())

        # 尝试采用蓝牙高速通信特性 0x46
        lzchar1.write_value(bytes([0x46]))

        # GPIO 上拉
        # lzchar1.write_value(bytes([0x27,0x10]))

        # 参数设置
        isCompassOn = 1  # 使用磁场融合姿态
        barometerFilter = 2
        Cmd_ReportTag = 0x67  # 功能订阅标识
        params = bytearray([0x00 for i in range(0, 11)])
        params[0] = 0x12
        params[1] = 5  # 静止状态加速度阀值
        params[2] = 255  # 静止归零速度(单位cm/s) 0:不归零 255:立即归零
        params[3] = 0  # 动态归零速度(单位cm/s) 0:不归零
        params[4] = ((barometerFilter & 3) << 1) | (isCompassOn & 1)
        params[5] = 80  # 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
        params[6] = 1  # 陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
        params[7] = 3  # 加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
        params[8] = 5  # 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
        params[9] = Cmd_ReportTag & 0xff
        params[10] = (Cmd_ReportTag >> 8) & 0xff
        #print("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
        time.sleep(2)
        lzchar1.write_value(params)

        # 主动上报 0x19
        lzchar1.write_value(bytes([0x19]))

        lzchar2 = next(
            c for c in service.characteristics
            if c.uuid == '0000ae02-0000-1000-8000-00805f9b34fb'.lower())
        lzchar2.enable_notifications()

    def descriptor_read_value_failed(self, descriptor, error):
        print('descriptor_value_failed')

    def characteristic_write_value_succeeded(self, characteristic):
        super().characteristic_write_value_succeeded(characteristic)
        print(buffer_name, "[%s] wr ok" % (self.mac_address))

    def characteristic_write_value_failed(self, characteristic, error):
        super().characteristic_write_value_failed(characteristic, error)
        #print("[%s] wr err %s" % (self.mac_address, error))

    def characteristic_enable_notifications_succeeded(self, characteristic):
        super().characteristic_enable_notifications_succeeded(characteristic)
        #print("[%s] notify ok" % (self.mac_address))

    def characteristic_enable_notifications_failed(self, characteristic, error):
        super().characteristic_enable_notifications_failed(characteristic, error)
        #print("[%s] notify err. %s" % (self.mac_address, error))

    def characteristic_value_updated(self, characteristic, value):
        # #print("Lzchar:", value.hex())
        # #print("value.size", len(value))
        # self.parse_imu(value)

        if characteristic.uuid == '0000ae02-0000-1000-8000-00805f9b34fb'.lower():
            if self.parse_imu_flage:
                self.parse_imu(value)
                self.imu_buffer.flush()

            if self.sock_pc is not None:
                #print("send blue source calibrate")
                self.sock_pc.sendall(value)

    # 这个是在本地解析
    def parse_imu(self, buf):
        scaleAccel = 0.00478515625  # 加速度 [-16g~+16g]    9.8*16/32768
        scaleQuat = 0.000030517578125  # 四元数 [-1~+1]         1/32768
        scaleAngle = 0.0054931640625  # 角度   [-180~+180]     180/32768
        scaleAngleSpeed = 0.06103515625  # 角速度 [-2000~+2000]    2000/32768
        scaleMag = 0.15106201171875  # 磁场 [-4950~+4950]   4950/32768
        scaleTemperature = 0.01  # 温度
        scaleAirPressure = 0.0002384185791  # 气压 [-2000~+2000]    2000/8388608
        scaleHeight = 0.0010728836  # 高度 [-9000~+9000]    9000/8388608

        imu_dat = array('f', [0.0 for i in range(0, 34)])
        
        if buf[0] == 0x11:
            ctl = (buf[2] << 8) | buf[1]
            #print(" subscribe tag: 0x%04x" % ctl)
            #print(" ms: ", ((buf[6] << 24) | (buf[5] << 16) | (buf[4] << 8) | (buf[3] << 0)))

            L = 7  # 从第7字节开始根据 订阅标识tag来解析剩下的数据
            if ((ctl & 0x0001) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel;
                L += 2
                #print("\taX: %.3f" % tmpX);  # x加速度aX
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel;
                L += 2
                #print("\taY: %.3f" % tmpY);  # y加速度aY
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel;
                L += 2
                #print("\taZ: %.3f" % tmpZ);  # z加速度aZ

                imu_dat[0] = float(tmpX)
                imu_dat[1] = float(tmpY)
                imu_dat[2] = float(tmpZ)
                if not use_Acc:
                    self.imu_buffer[0] = imu_dat[0]
                    self.imu_buffer[1] = imu_dat[1]
                    self.imu_buffer[2] = imu_dat[2]

            #print(" ")
            if ((ctl & 0x0002) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel;
                L += 2
                #print("\tAX: %.3f" % tmpX)  # x加速度AX
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel;
                L += 2
                #print("\tAY: %.3f" % tmpY)  # y加速度AY
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel;
                L += 2
                #print("\tAZ: %.3f" % tmpZ)  # z加速度AZ

                imu_dat[3] = float(tmpX)
                imu_dat[4] = float(tmpY)
                imu_dat[5] = float(tmpZ)
                if use_Acc:
                    self.imu_buffer[0] = imu_dat[3]
                    self.imu_buffer[1] = imu_dat[4]
                    self.imu_buffer[2] = imu_dat[5]


            #print(" ")
            if ((ctl & 0x0004) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngleSpeed;
                L += 2
                # #print("\tGX: %.3f" % tmpX)  # x角速度GX
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngleSpeed;
                L += 2
                # #print("\tGY: %.3f" % tmpY)  # y角速度GY
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngleSpeed;
                L += 2
                # #print("\tGZ: %.3f" % tmpZ)  # z角速度GZ

                imu_dat[6] = float(tmpX)
                imu_dat[7] = float(tmpY)
                imu_dat[8] = float(tmpZ)
                self.imu_buffer[3] = imu_dat[6]
                self.imu_buffer[4] = imu_dat[7]
                self.imu_buffer[5] = imu_dat[8]
            #print(" ")
            if ((ctl & 0x0008) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleMag;
                L += 2
                # #print("\tCX: %.3f" % tmpX);  # x磁场CX
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleMag;
                L += 2
                # #print("\tCY: %.3f" % tmpY);  # y磁场CY
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleMag;
                L += 2
                # #print("\tCZ: %.3f" % tmpZ);  # z磁场CZ

                imu_dat[9] = float(tmpX)
                imu_dat[10] = float(tmpY)
                imu_dat[11] = float(tmpZ)

            #print(" ")
            if ((ctl & 0x0010) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleTemperature;
                L += 2
                # #print("\ttemperature: %.2f" % tmpX)  # 温度

                tmpU32 = np.uint32(((np.uint32(buf[L + 2]) << 16) | (np.uint32(buf[L + 1]) << 8) | np.uint32(buf[L])))
                if ((tmpU32 & 0x800000) == 0x800000):  # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                    tmpU32 = (tmpU32 | 0xff000000)
                tmpY = np.int32(tmpU32) * scaleAirPressure;
                L += 3
                # #print("\tairPressure: %.3f" % tmpY);  # 气压

                tmpU32 = np.uint32((np.uint32(buf[L + 2]) << 16) | (np.uint32(buf[L + 1]) << 8) | np.uint32(buf[L]))
                if ((tmpU32 & 0x800000) == 0x800000):  # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                    tmpU32 = (tmpU32 | 0xff000000)
                tmpZ = np.int32(tmpU32) * scaleHeight;
                L += 3
                # #print("\theight: %.3f" % tmpZ);  # 高度

                imu_dat[12] = float(tmpX)
                imu_dat[13] = float(tmpY)
                imu_dat[14] = float(tmpZ)

            #print(" ")
            if ((ctl & 0x0020) != 0):
                tmpAbs = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleQuat;
                L += 2
                # #print("\tw: %.3f" % tmpAbs);  # w
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleQuat;
                L += 2
                # #print("\tx: %.3f" % tmpX);  # x
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleQuat;
                L += 2
                # #print("\ty: %.3f" % tmpY);  # y
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleQuat;
                L += 2
                # #print("\tz: %.3f" % tmpZ);  # z

                imu_dat[15] = float(tmpAbs)
                imu_dat[16] = float(tmpX)
                imu_dat[17] = float(tmpY)
                imu_dat[18] = float(tmpZ)
                self.imu_buffer[9] = imu_dat[16] # x
                self.imu_buffer[10] = imu_dat[17] # y
                self.imu_buffer[11] = imu_dat[18] # z
                self.imu_buffer[12] = imu_dat[15] # w

            #print(" ")
            if ((ctl & 0x0040) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngle;
                L += 2
                #print("\tangleX: %.3f" % tmpX);  # x角度
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngle;
                L += 2
                #print("\tangleY: %.3f" % tmpY);  # y角度
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngle;
                L += 2
                #print("\tangleZ: %.3f" % tmpZ);  # z角度

                print(color_head+"\r%s:X:%.3f,Y:%.3f,Z:%.3f"%(buffer_name, float(tmpX), float(tmpY), float(tmpZ))+color_tail,end='')
                imu_dat[19] = float(tmpX)
                imu_dat[20] = float(tmpY)
                imu_dat[21] = float(tmpZ)
                self.imu_buffer[6] = imu_dat[19]
                self.imu_buffer[7] = imu_dat[20]
                self.imu_buffer[8] = imu_dat[21]

            #print(" ")
            if ((ctl & 0x0080) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) / 1000.0;
                L += 2
                # #print("\toffsetX: %.3f" % tmpX);  # x坐标
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) / 1000.0;
                L += 2
                # #print("\toffsetY: %.3f" % tmpY);  # y坐标
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) / 1000.0;
                L += 2
                # #print("\toffsetZ: %.3f" % tmpZ);  # z坐标

                imu_dat[22] = float(tmpX)
                imu_dat[23] = float(tmpY)
                imu_dat[24] = float(tmpZ)
                

            #print(" ")
            if ((ctl & 0x0100) != 0):
                tmpU32 = ((buf[L + 3] << 24) | (buf[L + 2] << 16) | (buf[L + 1] << 8) | (buf[L] << 0));
                L += 4
                #print("\tsteps: %u" % tmpU32);  # 计步数
                tmpU8 = buf[L];
                L += 1
                if (tmpU8 & 0x01):  # 是否在走路
                    #print("\t walking yes")
                    imu_dat[25] = 100
                else:
                    #print("\t walking no")
                    imu_dat[25] = 0
                if (tmpU8 & 0x02):  # 是否在跑步
                    #print("\t running yes")
                    imu_dat[26] = 100
                else:
                    #print("\t running no")
                    imu_dat[26] = 0
                if (tmpU8 & 0x04):  # 是否在骑车
                    #print("\t biking yes")
                    imu_dat[27] = 100
                else:
                    #print("\t biking no")
                    imu_dat[27] = 0
                if (tmpU8 & 0x08):  # 是否在开车
                    #print("\t driving yes")
                    imu_dat[28] = 100
                else:
                    #print("\t driving no")
                    imu_dat[28] = 0

            #print(" ")
            if ((ctl & 0x0200) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel;
                L += 2
                # #print("\tasX: %.3f" % tmpX);  # x加速度asX
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel;
                L += 2
                # #print("\tasY: %.3f" % tmpY);  # y加速度asY
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel;
                L += 2
                # #print("\tasZ: %.3f" % tmpZ);  # z加速度asZ

                imu_dat[29] = float(tmpX)
                imu_dat[30] = float(tmpY)
                imu_dat[31] = float(tmpZ)

            #print(" ")
            if ((ctl & 0x0400) != 0):
                tmpU16 = ((buf[L + 1] << 8) | (buf[L] << 0));
                L += 2
                # #print("\tadc: %u" % tmpU16);  # adc测量到的电压值，单位为mv
                imu_dat[32] = float(tmpU16)

            #print(" ")
            if ((ctl & 0x0800) != 0):
                tmpU8 = buf[L];
                L += 1
                # #print("\t GPIO1  M:%X, N:%X" % ((tmpU8 >> 4) & 0x0f, (tmpU8) & 0x0f))
                imu_dat[33] = float(tmpU8)

        else:
            print(buffer_name, "[error] calibrate head not define")


# arg_parser = ArgumentParser(description="GATT Connect Demo")
# arg_parser.add_argument('mac_address', help="MAC address of device to connect")
# arg_parser.add_argument('host_ip', help="HOST ip address of device to connect", nargs='?', default=None)
# args = arg_parser.parse_args()


if __name__ == "__main__":
    rospy.init_node(buffer_name,anonymous=True)
    host_ip = "127.0.0.1"
    # host = args.host_ip
    host = None
    port = 7890
    sock = None
    #print("host ip: ", host)
    if host is not None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((host, port))
        except:
            #print("Could not make a connection to the server")
            input("Press enter to quit")
            sys.exit(0)

    #print("Connecting bluetooth ...")

    manager = gatt.DeviceManager(adapter_name='hci0')
    device = AnyDevice(manager=manager, mac_address=mac_address)
    device.sock_pc = sock
    if host is None:
        device.parse_imu_flage = True

    device.connect()

    manager.run()
