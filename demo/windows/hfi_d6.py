#!/usr/bin/env python
# -*- coding:utf-8 -*-
import math
import serial
import struct
import time
import sys
if sys.getdefaultencoding() != 'utf-8':
    reload(sys)
    sys.setdefaultencoding('utf-8')

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
    if (((byte_temp[0] + byte_temp[1] + byte_temp[2] + byte_temp[3] + byte_temp[4] + byte_temp[5] + byte_temp[6] +
          byte_temp[7] + byte_temp[8] + byte_temp[9]) & 0xff) == byte_temp[10]):
        return True
    else:
        print('sum check false!')
        return False


if __name__ == "__main__":

    try:
        hf_imu = serial.Serial(port='COM3', baudrate=9600, timeout=0.5)
        if hf_imu.isOpen():
            print u"imu connect success"
        else:
            hf_imu.open()
            print u"imu is open"

    except Exception, e:
        print e
        print u"找不到 ttyUSB0,请检查 ium 是否和电脑连接"
        exit()

    else:

        receive_buffer = bytearray()

        linear_acceleration_x = 0
        linear_acceleration_y = 0
        linear_acceleration_z = 0
        angular_velocity_x = 0
        angular_velocity_y = 0
        angular_velocity_z = 0

        while True:
            eul = []
            count = hf_imu.inWaiting()
            if count > 0:
                s = hf_imu.read(count)
                receive_buffer += s

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

                                if sb_sum_chech(byte_temp):
                                    Data = list(struct.unpack("hhhh", byte_temp[2:10]))
                                    # 加速度
                                    if byte_temp[1] == 0x51:
                                        linear_acceleration_x = Data[0] / 32768.0 * 16 * -9.8
                                        linear_acceleration_y = Data[1] / 32768.0 * 16 * -9.8
                                        linear_acceleration_z = Data[2] / 32768.0 * 16 * -9.8

                                    # 角速度
                                    if byte_temp[1] == 0x52:
                                        angular_velocity_x = Data[0] / 32768.0 * 2000 * math.pi / 180
                                        angular_velocity_y = Data[1] / 32768.0 * 2000 * math.pi / 180
                                        angular_velocity_z = Data[2] / 32768.0 * 2000 * math.pi / 180

                                    # 姿态角
                                    if byte_temp[1] == 0x53:
                                        angle_x = Data[0] / 32768.0 * 180
                                        angle_y = Data[1] / 32768.0 * 180
                                        angle_z = Data[2] / 32768.0 * 180

                                        print u'加速度:'
                                        print u'\t x轴加速度：' + "%.2f g" % linear_acceleration_x
                                        print u'\t y轴加速度：' + "%.2f g" % linear_acceleration_y
                                        print u'\t z轴加速度：' + "%.2f g" % linear_acceleration_z + "\r\n"

                                        print u'角速度：'
                                        print u'\t x轴角速度：' + "%.2f °/s" % angular_velocity_x
                                        print u'\t y轴角速度：' + "%.2f °/s" % angular_velocity_y
                                        print u'\t z轴角速度：' + "%.2f °/s" % angular_velocity_z + "\r\n"

                                        print u'角度：'
                                        print u'\t x轴角度：' + "%.2f °" % angle_x
                                        print u'\t y轴角度：' + "%.2f °" % angle_y
                                        print u'\t z轴角度：' + "%.2f °" % angle_z + "\r\n"
            time.sleep(0.001)

