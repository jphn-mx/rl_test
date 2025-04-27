import serial
import struct

class Port:
    def __init__(self,port_name,baudrate=230400):
        self.port_name = port_name
        self.baudrate = baudrate
        self.data = []

        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=1)
        except Exception as exc:
            print(f"串口打开异常: {exc}")
        
    def send_data(self,data):
        # 构造发送的帧头 (0x0a, 0x0b) 和帧尾 (0x0c)
        frame_head = bytes([0x00, 0x80, 0x3f])  # 帧头
        frame_tail = bytes([0xef,0xff,0xff])  # 帧尾

        # 将每个浮动点数转换为字节，使用 struct.pack 将其打包
        data_body = b''.join(struct.pack('f', x) for x in data)  # 'f' 格式表示浮点数

        # 在数据前后加上帧头和帧尾
        # full_data = frame_head + data_body + frame_tail
        full_data = data_body

        # 发送数据
        try:
            self.ser.write(full_data)
            print(f"发送成功，长度: {len(full_data)} 字节")
            print("发送数据(十六进制):", full_data.hex(' '))  # 打印发送的十六进制数据，用空格分隔
        except Exception as e:
            print(f"发送失败: {e}")
    
    def receive_data(self):
        try:
            # 读取数据
            received = self.ser.read(28)
            
            if received:
                # print(f"接收成功，长度: {len(received)} 字节")
                # print(received.hex(),end=" ")  # 打印接收的十六进制数据，用空格分隔
                
                # 这里可以添加数据解析逻辑
                # 例如检查帧头帧尾，提取数据部分等
                if len(received) >= 28:  # 至少包含帧头(2字节)和帧尾(1字节)
                    frame_head = received[:4]
                    frame_tail = received[-4:]
                    
                    if frame_head == bytes([0x00, 0x00, 0x48, 0x43]) and frame_tail == bytes([0x00, 0x00, 0xc8, 0x42]):
                        data_body = received[4:-4]
                        # 解析浮点数数据
                        if len(data_body) % 4 == 0:  # 每个浮点数占4字节
                            floats = []
                            for i in range(0, len(data_body), 4):
                                floats.append(struct.unpack('f', data_body[i:i+4])[0])
                            print("解析出的浮点数:", floats)
                        else:
                            print("数据体长度不符合浮点数格式")
                    else:
                        print("帧头或帧尾不匹配")
                else:
                    print("接收数据长度不足")
            else:
                print("未接收到数据")
                
            return received
        except Exception as e:
            print(f"接收失败: {e}")
            return None
            
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")

# if __name__ == "__main__":
#     p = Port("/dev/ttyACM0",9600)
#     while True:
#         # p.receive_data()
#         p.send_data([200.0,1.0,2.0,5.0,6.0,100.0])
