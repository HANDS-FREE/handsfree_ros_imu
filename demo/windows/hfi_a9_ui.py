# -*- coding:utf-8 -*-
import binascii
import math
import time
import struct
import serial
import threading
import Tkinter as tk
from datetime import datetime


# 配置类
class Config:
    # 端口号
    serialPort = 'COM3'
    # 波特率
    baudRate = 921600
    # 在传感器数据中，最小的包长度是11个字节
    minPackageLen = 11


# 传感器数据读取类
class SensorReader:
    def __init__(self):
        self.port = serial.Serial(Config.serialPort, Config.baudRate)
        self.port.close()
        if not self.port.isOpen():
            self.port.open()
        self.receiveBuffer = bytearray()
        self.working = False

    # 打开
    def open(self):
        if not self.port.isOpen():
            self.port.open()

    # 关闭
    def close(self):
        self.port.close()

    # 发送数据
    def send(self, data):
        self.port.write(data)

    # 接收数据
    def receive(self):
        while self.working:
            # 休眠一个微小的时间，可以避免无谓的CPU占用，在windows上实验时直接从12%下降到接近于0%
            time.sleep(0.001)
            count = self.port.inWaiting()
            if count > 24:
                self.receiveBuffer = binascii.b2a_hex(self.port.read(count))

    # 开始工作
    def start(self):
        # 开始数据读取线程
        t = threading.Thread(target=self.receive)
        # 将当前线程设为子线程t的守护线程，这样一来，当前线程结束时会强制子线程结束
        t.setDaemon(True)
        self.working = True
        t.start()

    # 停止工作
    def stop(self):
        self.working = False


# 数据解析类
class DataParser:
    def __init__(self, sensorReader, myUI):
        self.r = sensorReader
        self.u = myUI
        self.working = False
        self.TimeStart = datetime.now()
        self.iniVariable()

    def receiveSplit(self, receiveBuffer):
        buff = []
        for i in range(0, len(receiveBuffer), 2):
            buff.append(receiveBuffer[i:i + 2])
        return buff

    def hex_to_ieee(self, len, buff):
        str = ''
        data = []
        for i in range(len / 2 - 3, 11, -4):
            for j in range(i, i - 4, -1):
                str += buff[j]
            data.append(struct.unpack('>f', str.decode('hex'))[0])
            str = ''
        data.reverse()
        return data

    # 初始化解析丰关的变量
    def iniVariable(self):
        # self.ChipTime = [0, 0, 0, 0, 0, 0, 0]
        self.a = [0, 0, 0]
        self.w = [0, 0, 0]
        self.Angle = [0, 0, 0]
        self.h = [0, 0, 0]
        # self.Port = [0, 0, 0]
        # self.LastTime = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # self.Temperature = 0

    # 处理数据
    def handle(self):
        # 处理接收到的数据
        while self.working:
            # 显示当前收到的数据
            buff = self.receiveSplit(self.r.receiveBuffer)
            print buff
            if buff[0] + buff[1] + buff[2] == 'aa552c' and len(buff) == 49:
                sensor_data = self.hex_to_ieee(len(buff) * 2, buff)
                self.w[0] = sensor_data[0]
                self.w[1] = sensor_data[1]
                self.w[2] = sensor_data[2]
                self.a[0] = sensor_data[3] * -9.8
                self.a[1] = sensor_data[4] * -9.8
                self.a[2] = sensor_data[5] * -9.8
                self.h[0] = sensor_data[6] * 1000
                self.h[1] = sensor_data[7] * 1000
                self.h[2] = sensor_data[8] * 1000

            if buff[0] + buff[1] + buff[2] == 'aa5514' and len(buff) == 25:
                rpy = self.hex_to_ieee(len(buff) * 2, buff)
                self.Angle[0] = rpy[0]
                self.Angle[1] = - rpy[1]
                self.Angle[2] = - rpy[2] + 180

            # 内嵌的输出函数，可以直接引用方法内部的各种变量
            def output():
                text = '系统时间：' + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\r\n"
                # text += '相对时间：' + "%.3f" % TimeElapse + "\r\n\r\n"

                text += 'x轴加速度：' + "%.2f g" % self.a[0] + "\r\n"
                text += 'y轴加速度：' + "%.2f g" % self.a[1] + "\r\n"
                text += 'z轴加速度：' + "%.2f g" % self.a[2] + "\r\n\r\n"

                text += 'x轴角速度：' + "%.2f rad/s" % self.w[0] + "\r\n"
                text += 'y轴角速度：' + "%.2f rad/s" % self.w[1] + "\r\n"
                text += 'z轴角速度：' + "%.2f rad/s" % self.w[2] + "\r\n\r\n"

                text += 'x轴角度：  ' + "%.2f °" % self.Angle[0] + "\r\n"
                text += 'y轴角度：  ' + "%.2f °" % self.Angle[1] + "\r\n"
                text += 'z轴角度：  ' + "%.2f °" % self.Angle[2] + "\r\n\r\n"

                text += 'x轴磁场： ' + "%.0f mG" % self.h[0] + "\r\n"
                text += 'y轴磁场： ' + "%.0f mG" % self.h[1] + "\r\n"
                text += 'z轴磁场： ' + "%.0f mG" % self.h[2] + "\r\n\r\n"

                # text += '温   度：' + "%.2f ℃" % self.Temperature + "\r\n"
                self.u.showText(text)

            # 输出解析得到的内容
            output()

    # 开始工作
    def start(self):
        # 开启数据解析线程
        t = threading.Thread(target=self.handle)
        # 将当前线程设为子线程t的守护线程，这样一来，当前线程结束时会强制子线程结束
        t.setDaemon(True)
        self.working = True
        t.start()

    # 停止工作
    def stop(self):
        self.working = False


# 图形界面类
class MyUI:
    def __init__(self):
        # 创建显示传感器数据的窗口
        self.window = tk.Tk()
        self.window.title('HFI-A9')
        self.window.geometry('640x360')

        self.frameBoy = tk.Frame(self.window)
        self.frameBoy.config(height=345, width=625)
        self.frameBoy.place(x=5, y=5)

        self.textBox = tk.Text(self.frameBoy, height=700, bg='white', font=('Arial', 12))
        self.textBox.place(x=4, y=4)

    # 开启UI
    def start(self):
        # 开启窗口主循环
        self.window.mainloop()

    # 显示文本
    def showText(self, text):
        self.textBox.delete(0.0, tk.END)
        self.textBox.insert(tk.INSERT, text)


# 主线程
if __name__ == '__main__':
    # 创建串口操作对象
    r = SensorReader()
    # 开始读取数据
    r.start()

    # 创建UI对象
    u = MyUI()

    # 创建数据解析对象
    p = DataParser(r, u)

    # 开始解析数据
    p.start()

    # 启动UI
    u.start()
