#!/usr/bin/env python
import cv2
import serial
import rospy
from std_msgs.msg import Bool, String
from pyzbar.pyzbar import decode
from ac_control.msg import qr_scanner  # 导入自定义消息类型

class QRScanner:
    def __init__(self):
        # 初始化节点、硬件和状态变量
        self.init_ros()
        self.init_hardware()
        self.init_state()
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("节点启动完成,进入模式1")

    def init_ros(self):
        rospy.init_node('qr_scanner_node')
        self.pub = rospy.Publisher('/vision_node/qr_scan/statue', qr_scanner, queue_size=1)
        rospy.Subscriber('/fcu_node/scan/state', Bool, self.callback_state, queue_size=1)
        rospy.Subscriber('/fcu_node/location', String, self.callback_location, queue_size=1)

    def init_hardware(self):
        self.ser = self.init_serial('/dev/ttyUSB0', 115200)
        self.cap_index = '/dev/camera_right'  # 初始摄像头索引
        self.cap = self.init_camera(self.cap_index)

    def init_state(self):
        self.ACK = False
        self.flag = False
        self.location = None
        self.target_position = None
        self.index = 0
        self.array = [[0 for _ in range(7)] for _ in range(4)]
        self.scanned_data = set()  # 用于存储已扫描的二维码数据
        self.current_mode = "mode1"
        self.target_qr = None
        self.cnt = 0  # 计数器，用于统计不重复二维码数量
        self.x = 0  # 初始化 x 坐标
        self.y = 0  # 初始化 y 坐标
    def init_serial(self, port, baudrate):
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            rospy.loginfo(f"串口初始化成功: {port}")
            return ser
        except Exception as e:
            rospy.logerr(f"串口初始化失败: {str(e)}")
            rospy.signal_shutdown("硬件错误")
            return None

    def init_camera(self, index):
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            rospy.logerr("摄像头初始化失败！")
            rospy.signal_shutdown("硬件错误")
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        rospy.loginfo(f"摄像头 {index} 初始化成功")
        return cap

    def switch_camera(self):
        """切换摄像头"""
        if self.current_mode == "mode1":
            self.cap_index = '/dev/camera_left' if self.cap_index =='/dev/camera_right'  else '/dev/camera_right'  # 在 0 和 1 之间切换
            self.cap.release()  # 释放当前摄像头
            self.cap = self.init_camera(self.cap_index)  # 初始化新的摄像头
            rospy.loginfo(f"切换到摄像头 {self.cap_index}")
        if self.current_mode == "mode2":
            if self.target_position=='A' or self.target_position=='C':
                self.cap_index = '/dev/camera_right' 
            else:
                self.cap_index = '/dev/camera_left' 
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.check_serial_cmd()
            self.process_frame()

    def check_serial_cmd(self):
        if self.ser and self.ser.in_waiting > 0:
            cmd = self.ser.readline().decode().strip().lower()
            if cmd in ["mode1", "mode2", "mode3"]:
                self.current_mode = cmd
                if cmd == "mode1":
                    self.target_qr = None
                rospy.loginfo(f"切换到{cmd}模式")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("帧读取失败")
            return

        decoded = decode(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if decoded:
            qr_data = decoded[0].data.decode('utf-8').zfill(2)
            self.handle_qr_data(qr_data, frame)

        cv2.imshow("QR Scanner", frame)
        cv2.waitKey(1)

    def handle_qr_data(self, qr_data, frame):
        """处理二维码数据"""
        if qr_data not in self.scanned_data and self.array[self.x][self.y] != 0:
            pass
        else :
            self.publish_data(qr_data)
            
        #上面这段逻辑是防止出现还没到达下一个二维码位置就已经扫描到二维码的情况
        if self.current_mode == "mode2":
            self.send_serial(f"A{qr_data}T")
            self.target_qr = qr_data
            self.switch_camera()
        elif self.current_mode == "mode3":
            self.send_serial(f"A{qr_data}T")
            if qr_data == self.target_qr:
                self.send_serial("success")
                rospy.loginfo("匹配成功: success")
        else:
            # 保证当前位置二维码数据不被下一节点的覆盖
            if self.array[self.x][self.y] != 0:
                rospy.loginfo(f"当前位置 {self.location} 已存储过二维码数据，跳过存储，但继续发送串口信息")
                #因为可嫩还没到下一个节点
            elif self.location is None:
                rospy.logwarn("当前位置未设置，无法存储二维码数据")
            else:
                for row in self.array:
                    if qr_data in row:
                        rospy.loginfo(f"二维码数据 {qr_data} 已存在，跳过存储，但继续发送串口信息")
                        break
                
                else:
                    self.array[self.x][self.y] = qr_data
                    self.scanned_data.add(qr_data)
                    self.ACK=True
                    self.cnt += 1  # 增加计数器
                    
                    #每扫描到6个不重复的二维码后切换摄像头
                if self.cnt >= 6:
                    self.cnt = 0  # 清零计数器
                    self.switch_camera()
                    rospy.loginfo(f"存储二维码数据 {qr_data} 到位置 {self.location}")
                    
        
        # self.draw_markers(frame, qr_data)
        self.send_serial(f"A{qr_data}{self.location}T")
    def draw_markers(self, frame, text):
        """在帧上绘制标记信息"""
        cv2.putText(frame, f"Mode: {self.current_mode}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if self.current_mode == "mode2" and self.target_qr:
            cv2.putText(frame, f"Target: {self.target_qr}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    def publish_data(self, data):
        msg = qr_scanner()
        msg.state = self.ACK
        msg.mode = 1 if self.current_mode == "mode1" else 2
        if msg.mode == 2:
            row, col = self.find_position(self.array, data)
            msg.message = data
            msg.position = f"({chr(row + 65)}{col})"
            self.target_position= f"({chr(row + 65)})"
        else:
            msg.message = data
        self.pub.publish(msg)

    def callback_state(self, msg):
        if msg.data:
            rospy.loginfo("接收到fcu_node/scan/state消息,开始等待下一个二维码扫描")
            self.ACK = False

    def callback_location(self, msg):
        self.flag = True
        self.location = msg.data
        self.x = ord(msg.data[0]) - ord('A')
        self.y = int(msg.data[1])

    def send_serial(self, message):
        if self.ser:
            try:
                self.ser.write(message.encode())
                rospy.logdebug(f"串口发送: {message}")
            except Exception as e:
                rospy.logerr(f"串口发送失败: {str(e)}")

    def cleanup(self):
        rospy.loginfo("释放资源...")
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        if self.ser:
            self.ser.close()
        rospy.loginfo(f"扫描记录: {sorted(self.scanned_data)}")

if __name__ == '__main__':
    try:
        scanner = QRScanner()
        scanner.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass