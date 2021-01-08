#!/usr/bin/env python
# -*- coding:utf-8 -*-
import math
import serial
import struct
import time
import rospy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

cov_orientation = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cov_angular_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cov_linear_acceleration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def eul_to_qua(Eular):
    eular_div = [0, 0, 0]
    eular_div[0], eular_div[1], eular_div[2] = Eular[0] / 2.0, Eular[1] / 2.0, Eular[2] / 2.0

    ca, cb, cc = math.cos(eular_div[0]), math.cos(eular_div[1]), math.cos(eular_div[2])
    sa, sb, sc = math.sin(eular_div[0]), math.sin(eular_div[1]), math.sin(eular_div[2])

    x = sa * cb * cc - ca * sb * sc
    y = ca * sb * cc + sa * cb * sc
    z = ca * cb * sc - sa * sb * cc
    w = ca * cb * cc + sa * sb * sc

    orientation = Quaternion()
    orientation.x, orientation.y, orientation.z, orientation.w = x, y, z, w
    return orientation


# 在缓冲数据中找到第一个包的起始位置
def find_first_package(buffer):
    i = 0
    while True:
        if buffer[i] == 0x55 and (buffer[i + 1] & 0x50) == 0x50:
            return i
        if i + 2 >= len(buffer):
            return -1
        i += 1


# 检查校验和
def sb_sum_chech(byte_temp):
    # if(len(byte_temp)==11) :
        if (((byte_temp[0] + byte_temp[1] + byte_temp[2] + byte_temp[3] + byte_temp[4] + byte_temp[5] + byte_temp[6] +
              byte_temp[7] + byte_temp[8] + byte_temp[9]) & 0xff) == byte_temp[10]):
            # print('sum check ok!')
            return True
        else:
            print('sum check false!')
            return False


if __name__ == "__main__":
    rospy.init_node("imu")

    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baudrate", 921600)

    try:
        hf_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if hf_imu.isOpen():
            rospy.loginfo("imu connect success")
        else:
            hf_imu.open()
            rospy.loginfo("imu is open")

    except Exception, e:
        print e
        rospy.loginfo("找不到 ttyUSB0,请检查 ium 是否和电脑连接")
        exit()

    else:
        imu_pub = rospy.Publisher("handsfree/imu", Imu, queue_size=10)

        receive_buffer = bytearray()

        linear_acceleration_x = 0
        linear_acceleration_y = 0
        linear_acceleration_z = 0
        angular_velocity_x = 0
        angular_velocity_y = 0
        angular_velocity_z = 0

        while not rospy.is_shutdown():
            eul = []
            count = hf_imu.inWaiting()
            if count > 0:
                s = hf_imu.read(count)
                receive_buffer += s
            stamp = rospy.get_rostime()
            dataLen = len(receive_buffer)
            if dataLen >= 11:
                # 去掉第1个包头前的数据
                headerPos = find_first_package(receive_buffer)
                if headerPos >= 0:
                    if headerPos > 0:
                        receive_buffer[0:headerPos] = b''
                    # 取 Config.minPackageLen 整数倍长度的数据
                    if dataLen - headerPos >= 11:
                        packageCount = int((dataLen - headerPos) / 11)
                        if packageCount > 0:
                            cutLen = packageCount * 11
                            temp = receive_buffer[0:cutLen]
                            # 按16进制字符串的形式显示收到的内容
                            receive_buffer[0:cutLen] = b''

                            # 解析数据,逐个数据包进行解析
                            for i in range(packageCount):
                                beginIdx = int(i * 11)
                                endIdx = int(i * 11 + 11)
                                byte_temp = temp[beginIdx:endIdx]
                                # 校验和通过了的数据包才进行解析
                                imu_msg = Imu()
                                imu_msg.header.stamp = stamp
                                imu_msg.header.frame_id = "base_link"
                                mag_msg = MagneticField()
                                mag_msg.header.stamp = stamp
                                mag_msg.header.frame_id = "base_link"

                                if sb_sum_chech(byte_temp):
                                    Data = list(struct.unpack("hhhh", byte_temp[2:10]))
                                    # 加速度
                                    if byte_temp[1] == 0x51:
                                        linear_acceleration_x = Data[0] / 32768.0 * 16 * -9.8
                                        linear_acceleration_y = Data[1] / 32768.0 * 16 * -9.8
                                        linear_acceleration_z = Data[2] / 32768.0 * 16 * -9.8

                                    # 角速度
                                    if byte_temp[1] == 0x52:
                                        imu_msg.orientation_covariance = cov_orientation
                                        angular_velocity_x = Data[0] / 32768.0 * 2000 * math.pi / 180
                                        angular_velocity_y = Data[1] / 32768.0 * 2000 * math.pi / 180
                                        angular_velocity_z = Data[2] / 32768.0 * 2000 * math.pi / 180

                                    # 姿态角
                                    if byte_temp[1] == 0x53:
                                        for i in range(3):
                                            eul.append(Data[i]/32768.0 * math.pi)

                                        imu_msg.orientation = eul_to_qua(eul)

                                        imu_msg.linear_acceleration.x = linear_acceleration_x
                                        imu_msg.linear_acceleration.y = linear_acceleration_y
                                        imu_msg.linear_acceleration.z = linear_acceleration_z

                                        imu_msg.angular_velocity.x = angular_velocity_x
                                        imu_msg.angular_velocity.y = angular_velocity_y
                                        imu_msg.angular_velocity.z = angular_velocity_z

                                        imu_msg.angular_velocity_covariance = cov_angular_velocity
                                        imu_msg.linear_acceleration_covariance = cov_linear_acceleration

                                        imu_pub.publish(imu_msg)
            time.sleep(0.001)
