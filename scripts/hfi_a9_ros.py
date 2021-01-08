#!/usr/bin/env python
# -*- coding:utf-8 -*-
import binascii
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
cov_magnetic_field = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


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


def receive_split(receive_buffer):
    buff = []
    for i in range(0, len(receive_buffer), 2):
        buff.append(receive_buffer[i:i + 2])
    return buff


def hex_to_ieee(len, buff):
    str = ''
    data = []
    for i in range(len / 2 - 3, 11, -4):
        for j in range(i, i - 4, -1):
            str += buff[j]
        data.append(struct.unpack('>f', str.decode('hex'))[0])
        str = ''
    data.reverse()
    return data


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
        mag_pub = rospy.Publisher("handsfree/mag", MagneticField, queue_size=10)
        sensor_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        while not rospy.is_shutdown():
            count = hf_imu.inWaiting()
            if count > 24:
                # bytearray() 方法返回一个新字节数组。这个数组里的元素是可变的，并且每个元素的值范围: 0 <= x < 256
                receive_buffer = bytearray()
                receive_buffer = binascii.b2a_hex(hf_imu.read(count))
                receive_len = len(receive_buffer)
                stamp = rospy.get_rostime()
                buff = receive_split(receive_buffer)

                if buff[0]+buff[1]+buff[2] == 'aa552c':
                    sensor_data = hex_to_ieee(receive_len, buff)
                rpy_degree = []

                if buff[0]+buff[1]+buff[2] == 'aa5514':
                    rpy = hex_to_ieee(receive_len, buff)
                    rpy_degree.append(rpy[0] / 180 * math.pi)
                    rpy_degree.append(rpy[1] / -180 * math.pi)
                    rpy_degree.append(rpy[2] / -180 * math.pi)

                    imu_msg = Imu()

                    imu_msg.header.stamp = stamp
                    imu_msg.header.frame_id = "base_link"

                    # 调用 eul_to_qua , 将欧拉角转四元数
                    imu_msg.orientation = eul_to_qua(rpy_degree)
                    imu_msg.orientation_covariance = cov_orientation

                    imu_msg.angular_velocity.x = sensor_data[0]
                    imu_msg.angular_velocity.y = sensor_data[1]
                    imu_msg.angular_velocity.z = sensor_data[2]
                    imu_msg.angular_velocity_covariance = cov_angular_velocity

                    imu_msg.linear_acceleration.x = sensor_data[3] * -9.8
                    imu_msg.linear_acceleration.y = sensor_data[4] * -9.8
                    imu_msg.linear_acceleration.z = sensor_data[5] * -9.8
                    imu_msg.linear_acceleration_covariance = cov_linear_acceleration

                    imu_pub.publish(imu_msg)

                    mag_msg = MagneticField()
                    mag_msg.header.stamp=stamp
                    mag_msg.header.frame_id="base_link"
                    mag_msg.magnetic_field.x = sensor_data[6]
                    mag_msg.magnetic_field.y = sensor_data[7]
                    mag_msg.magnetic_field.z = sensor_data[8]
                    mag_msg.magnetic_field_covariance = cov_magnetic_field

                    mag_pub.publish(mag_msg)

            time.sleep(0.001)

