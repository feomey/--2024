#!/usr/bin/env python3
import cv2
import serial
import time
import rospy
from std_msgs.msg import Bool
from ac_control.msg import qr_scanner  # 导入自定义消息类型
def callback(msg):
    if msg.state == True:
        rospy.loginfo("接收到fcu_node/scan/state消息,开始等待下一个二维码扫描")
        msg=Bool()
        msg.data = False
        pub.publish(msg)
        

if __name__ == "__main__":
    rospy.init_node('receive_node')
    pub = rospy.Publisher('fcu_node/scan/state', Bool, queue_size=1)
    scr = rospy.Subscriber('/vision_node/qr_scan/statue', qr_scanner, callback, queue_size=1)
    rospy.spin()

   
