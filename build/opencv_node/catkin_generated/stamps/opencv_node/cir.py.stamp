#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from ac_control.msg import cir_position  # 自定义消息类型

# 参数配置
MIN_AREA = 500        # 最小轮廓面积
MAX_AREA = 100000      # 最大轮廓面积
RESOLUTION = (640, 480)  # 处理分辨率

class CircleDetectorROS:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('circle_detector_node', anonymous=True)
        self.pub = rospy.Publisher('/vision_node/cir_scan', cir_position, queue_size=1)
        self.cap = self.init_camera(1)
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("圆检测节点启动成功")

    def init_camera(self, index):
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            rospy.logerr("摄像头初始化失败！")
            rospy.signal_shutdown("硬件错误")
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
        rospy.loginfo("摄像头初始化成功")
        return cap

    def optimize_detection(self, frame):
        """优化的预处理管道"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 1)
        edges = cv2.Canny(blur, 50, 150)
        return edges

    def detect_circle(self, img_contour, cnt):
        """圆形检测主函数"""
        area = cv2.contourArea(cnt)
        if not (MIN_AREA < area < MAX_AREA):
            return None
        
        # 使用最小外接圆检测圆形
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        area_ratio = area / (np.pi * radius**2 + 1e-5)
        if 0.8 < area_ratio < 1.2:  # 面积比验证
            cv2.circle(img_contour, (int(cx), int(cy)), int(radius), (0, 0, 255), 2)
            cv2.putText(img_contour, f"Circle", (int(cx)-40, int(cy)-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
            return (int(cx), int(cy), int(radius))  # 返回圆心和半径
        
        return None

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("帧读取失败")
                continue
            
            frame = cv2.resize(frame, RESOLUTION)
            processed = self.optimize_detection(frame)
            contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            result = frame.copy()
            for cnt in contours:
                circle = self.detect_circle(result, cnt)
                if circle:
                    cx, cy, radius = circle
                    rospy.loginfo(f"检测到圆: 圆心=({cx}, {cy}), 半径={radius}")

                    # 发布ROS消息
                    msg = cir_position()
                    msg.x = cx
                    msg.y = cy
                    
                    self.pub.publish(msg)

            # 显示实时FPS
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            cv2.putText(result, f"FPS: {int(fps)}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            
            cv2.imshow("Circle Detection", result)
            if cv2.waitKey(1) == ord('q'):
                break

            rate.sleep()

    def cleanup(self):
        rospy.loginfo("释放资源...")
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = CircleDetectorROS()
        detector.run()
    except rospy.ROSInterruptException:
        pass