# -*- coding:utf-8 -*-
import time
import struct
import serial
import threading
import Tkinter as tk
from datetime import datetime


# 配置类
class Config:
    # 端口号
    serialPort = '/dev/ttyUSB0'
    # 波特率
    baudRate = 9600
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
            if count > 0:
                s = self.port.read(count)
                self.receiveBuffer += s

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

    # 流逝的毫秒数，返回浮点数
    def elapseMilliSeconds(self):
        now = datetime.now()
        seconds = time.mktime(now.timetuple()) - time.mktime(self.TimeStart.timetuple())
        # 微秒数的差值
        microSeconds = now.microsecond - self.TimeStart.microsecond
        return seconds * 1000 + microSeconds / 1000.0

    # 流逝的秒数，返回浮点数
    def elapseSeconds(self):
        return self.elapseMilliSeconds() / 1000

    # 在缓冲数据中找到第一个包的起始位置
    def findFirstPackage(self, buffer):
        i = 0
        while True:
            if buffer[i] == 0x55 and (buffer[i + 1] & 0x50) == 0x50:
                return i
            if i + 2 >= len(buffer):
                return -1
            i += 1

    # 处理数据
    def handle(self):
        # 处理接收到的数据
        while self.working:
            # 显示当前收到的数据
            dataLen = len(self.r.receiveBuffer)
            if dataLen >= Config.minPackageLen:
                # 去掉第1个包头前的数据
                headerPos = self.findFirstPackage(self.r.receiveBuffer)

                if headerPos >= 0:
                    if headerPos > 0:
                        self.r.receiveBuffer[0:headerPos] = b''
                    # 取 Config.minPackageLen 整数倍长度的数据
                    if dataLen - headerPos >= Config.minPackageLen:
                        packageCount = int((dataLen - headerPos) / Config.minPackageLen)
                        if packageCount > 0:
                            cutLen = packageCount * Config.minPackageLen

                            temp = self.r.receiveBuffer[0:cutLen]
                            self.r.receiveBuffer[0:cutLen] = b''

                            # 解析数据,逐个数据包进行解析
                            for i in range(packageCount):
                                beginIdx = int(i * Config.minPackageLen)
                                endIdx = int(i * Config.minPackageLen + Config.minPackageLen)
                                byteTemp = temp[beginIdx:endIdx]
                                # 校验和通过了的数据包才进行解析
                                if self.sbSumCheck(byteTemp):
                                    self.decodeData(byteTemp)
            time.sleep(0.005)

    # 初始化解析丰关的变量
    def iniVariable(self):
        self.ChipTime = [0, 0, 0, 0, 0, 0, 0]
        self.a = [0, 0, 0, 0]
        self.w = [0, 0, 0, 0]
        self.Angle = [0, 0, 0, 0]
        self.h = [0, 0, 0, 0]
        self.Port = [0, 0, 0, 0]
        self.LastTime = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.Temperature = 0

    # 解码包中的数据
    def decodeData(self, byteTemp):
        # 记录当前的相对时间
        TimeElapse = self.elapseSeconds()
        # 将8个字节的数据解析成4个短整型
        Data = list(struct.unpack("hhhh", byteTemp[2:10]))

        # 时间
        if byteTemp[1] == 0x50:
            self.ChipTime[0] = (2000 + byteTemp[2])
            self.ChipTime[1] = byteTemp[3]
            self.ChipTime[2] = byteTemp[4]
            self.ChipTime[3] = byteTemp[5]
            self.ChipTime[4] = byteTemp[6]
            self.ChipTime[5] = byteTemp[7]
            self.ChipTime[6] = struct.unpack("h", byteTemp[8:10])[0]

        # 加速度
        if byteTemp[1] == 0x51:
            self.Temperature = Data[3] / 100.0
            Data[0] = Data[0] / 32768.0 * 16
            Data[1] = Data[1] / 32768.0 * 16
            Data[2] = Data[2] / 32768.0 * 16

            self.a[0] = Data[0]
            self.a[1] = Data[1]
            self.a[2] = Data[2]
            self.a[3] = Data[3]
            if ((TimeElapse - self.LastTime[1]) < 0.1):
                return
            self.LastTime[1] = TimeElapse

        # 角速度
        if byteTemp[1] == 0x52:
            self.Temperature = Data[3] / 100.0
            Data[0] = Data[0] / 32768.0 * 2000
            Data[1] = Data[1] / 32768.0 * 2000
            Data[2] = Data[2] / 32768.0 * 2000
            self.w[0] = Data[0]
            self.w[1] = Data[1]
            self.w[2] = Data[2]
            self.w[3] = Data[3]
            if ((TimeElapse - self.LastTime[2]) < 0.1):
                return
            self.LastTime[2] = TimeElapse

        # 姿态角
        if byteTemp[1] == 0x53:
            self.Temperature = Data[3] / 100.0
            Data[0] = Data[0] / 32768.0 * 180
            Data[1] = Data[1] / 32768.0 * 180
            Data[2] = Data[2] / 32768.0 * 180
            self.Angle[0] = Data[0]
            self.Angle[1] = Data[1]
            self.Angle[2] = Data[2]
            self.Angle[3] = Data[3]
            if ((TimeElapse - self.LastTime[3]) < 0.1):
                return
            self.LastTime[3] = TimeElapse

        # 磁场
        if byteTemp[1] == 0x54:
            self.Temperature = Data[3] / 100.0
            self.h[0] = Data[0]
            self.h[1] = Data[1]
            self.h[2] = Data[2]
            self.h[3] = Data[3]
            if ((TimeElapse - self.LastTime[4]) < 0.1):
                return
            self.LastTime[4] = TimeElapse

        # 端口状态数据输出
        if byteTemp[1] == 0x55:
            self.Port[0] = Data[0]
            self.Port[1] = Data[1]
            self.Port[2] = Data[2]
            self.Port[3] = Data[3]

        # 内嵌的输出函数，可以直接引用方法内部的各种变量，比如 TimeElapse 等
        def output():
            text = '波特率：9600' + "\r\n"
            text += '频率：20Hz' + "\r\n\r\n"

            text += '系统时间：' + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\r\n"
            text += '相对时间：' + "%.3f" % TimeElapse + "\r\n\r\n"

            text += 'x轴加速度：' + "%.2f g" % self.a[0] + "\r\n"
            text += 'y轴加速度：' + "%.2f g" % self.a[1] + "\r\n"
            text += 'z轴加速度：' + "%.2f g" % self.a[2] + "\r\n\r\n"

            text += 'x轴角速度：' + "%.2f °/s" % self.w[0] + "\r\n"
            text += 'y轴角速度：' + "%.2f °/s" % self.w[1] + "\r\n"
            text += 'z轴角速度：' + "%.2f °/s" % self.w[2] + "\r\n\r\n"

            text += 'x轴角度：  ' + "%.2f °" % self.Angle[0] + "\r\n"
            text += 'y轴角度：  ' + "%.2f °" % self.Angle[1] + "\r\n"
            text += 'z轴角度：  ' + "%.2f °" % self.Angle[2] + "\r\n\r\n"

            text += 'x轴磁场： ' + "%.0f mG" % self.h[0] + "\r\n"
            text += 'y轴磁场： ' + "%.0f mG" % self.h[1] + "\r\n"
            text += 'z轴磁场： ' + "%.0f mG" % self.h[2] + "\r\n\r\n"

            text += '温   度：' + "%.2f ℃" % self.Temperature + "\r\n"

            self.u.showText(text)

        # 输出解析得到的内容
        output()

    # 检查校验和
    def sbSumCheck(self, byteTemp):
        if (((byteTemp[0] + byteTemp[1] + byteTemp[2] + byteTemp[3] + byteTemp[4] + byteTemp[5] + byteTemp[6] +
              byteTemp[7] + byteTemp[8] + byteTemp[9]) & 0xff) == byteTemp[10]):
            return True
        else:
            print('sum check false!')
            return False

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
        self.window.title('HFI-D6')
        self.window.geometry('750x450')

        self.frameBoy = tk.Frame(self.window)
        self.frameBoy.config(height=435, width=735)
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
