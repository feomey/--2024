#!/usr/bin/env python
import cv2
import serial
import time
import rospy
from std_msgs.msg import String


class QRScanner:
    def __init__(self):
        # ROS初始化：初始化节点，摄像头和串口通信
        self.ACK=False
        rospy.init_node('qr_scanner_node')
        self.qr_pub = rospy.Publisher('/qr_code_data', String, queue_size=10)
        
        # 硬件初始化
        self.ser = self.init_serial('/dev/ttyUSB0', 115200)
        self.cap = self.init_camera(0)
        self.detector = cv2.QRCodeDetector()
        
        # 运行状态
        self.scanned_data = set()
        self.current_mode = "mode1"  # 默认模式
        self.target_qr = None
        self.waiting_start = 0
        
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("节点启动完成，进入模式1")


#串口初始化函数
    def init_serial(self, port, baudrate):
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            rospy.loginfo(f"串口初始化成功: {port}")
            return ser
        except Exception as e:
            rospy.logerr(f"串口初始化失败: {str(e)}")
            rospy.signal_shutdown("硬件错误")
            return None

#相机初始化
    def init_camera(self, index):
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            rospy.logerr("摄像头初始化失败！")
            rospy.signal_shutdown("硬件错误")
            return None
        
        # 设置摄像头参数（根据实际硬件调整）
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        rospy.loginfo("摄像头初始化成功")
        return cap

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.check_serial_cmd()             #检查串口命令
            self.process_frame()
            # rate.sleep()


#用来切换模式的，切换mode2时会把上次的mode2设置的目标清空
    def check_serial_cmd(self):
        if self.ser and self.ser.in_waiting > 0:
            cmd = self.ser.readline().decode().strip().lower()
            if cmd in ["mode1", "mode2","mode3"]:
                self.current_mode = cmd
                if cmd == "mode1":
                    self.target_qr = None
                rospy.loginfo(f"切换到{cmd}模式")

#总处理函数：读取二维码信息，再通过handle_qr_data处理
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("帧读取失败")
            return

        data, bbox, _ = self.detector.detectAndDecode(frame)
        if data:
            self.handle_qr_data(data.zfill(2), frame)

        cv2.imshow("QR Scanner", frame)
        cv2.waitKey(1)

#通过前面的切换模式来进入不同的处理逻辑
    def handle_qr_data(self, qr_data, frame):
        
        rospy.loginfo(f"二维码: {qr_data}")
        self.send_serial(f"A{qr_data}T")
        if self.current_mode == "mode2":    
            self.target_qr = qr_data
            print(self.current_mode)
        elif self.current_mode == "mode3":  
            if  qr_data==self.target_qr:
                self.send_serial("success")
                rospy.loginfo("匹配成功: success")  
                print(self.current_mode)      
        else:
            self.send_serial(f"A{qr_data}T")
            if qr_data not in self.scanned_data:
                self.scanned_data.add(qr_data)
                self.publish_data(qr_data)
                print(self.current_mode)
        
        self.draw_markers(frame, qr_data)

#模式2进行匹配
    def process_mode2_logic(self, qr_data):
        if self.target_qr and qr_data == self.target_qr:
                self.send_serial("success")
                rospy.loginfo("匹配成功: success")

    def publish_data(self, data):
        msg = String()
        msg.data = data
        self.qr_pub.publish(msg)

    def send_serial(self, message):
        if self.ser:
            try:
                self.ser.write(message.encode())
                rospy.logdebug(f"串口发送: {message}")
            except Exception as e:
                rospy.logerr(f"串口发送失败: {str(e)}")
#
    def draw_markers(self, frame, text):
        cv2.putText(frame, f"Mode: {self.current_mode}", (10,30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        if self.current_mode == "mode2" and self.target_qr:
            cv2.putText(frame, f"Target: {self.target_qr}", (10,70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
#释放资源
    def cleanup(self):
        rospy.loginfo("释放资源...")
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()
        rospy.loginfo(f"扫描记录: {sorted(self.scanned_data)}")

if __name__ == '__main__':
    try:
        scanner = QRScanner()
        scanner.run()
    except rospy.ROSInterruptException:
        pass