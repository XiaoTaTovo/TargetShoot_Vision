import serial
class SerialManager:
    def __init__(self,port='COM3',baudrate=115200):
        self.ser=None#初始化串口对象为None,self.让变量在函数运行结束后不被回收,让同一个类里的不同函数（比如 __init__、send、close）能共同操作同一个串口。
        try:
            self.ser = serial.Serial(port,baudrate,timeout=0.1)
            print(f"成功连接到串口 {port}")
        except Exception as e:
            print(f"无法连接到串口 {port},错误: {e}")
    def send_gimbal_data(self, err_x, err_y, state):
        """
        发送云台追踪数据 (字符串协议)
        :param err_x: X轴像素偏差 (例如 -150 到 +150)
        :param err_y: Y轴像素偏差
        :param state: 1表示锁定目标，0表示丢失目标
        """
        if self.ser is None or not self.ser.is_open:
            return
            
        # 格式化字符串，强制带符号，并补齐3位数字。
        # 结果类似于: <X:-120,Y:+045,S:1>\n
        packet_str = f"<X:{int(err_x):+04d},Y:{int(err_y):+04d},S:{state}>\n"
        
        try:
            # 字符串必须编码成字节才能通过串口发送
            self.ser.write(packet_str.encode('utf-8'))
            
            # 调试打印 (实际上机时建议注释掉)
            print(f"发送数据: {packet_str.strip()}")
        except Exception as e:
            print(f"串口发送失败: {e}")
        
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")