#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
import time

class MazeNavigator:
    def __init__(self):
        rospy.init_node('maze_navigator', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/rrbot/laser/scan', LaserScan, self.laser_callback)
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
        self.twist = Twist()
        self.bridge = CvBridge()
        self.right_wall_distance = 0.3  # 設定右側牆壁的目標距離為0.3公尺
        self.stop_movement = False
        self.last_stop_time = 0
        self.front_distance = 10

    def laser_callback(self, data):
        if self.stop_movement:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            return

        # 獲取雷達數據
        right_dist = min(min(data.ranges[0:60]), 10)  # 右側距離
        front_dist = min(min(data.ranges[300:420]), 10)  # 前方距離
        self.front_distance = front_dist

        if front_dist < 0.5:
            # 前方有障礙物，左轉
            self.twist.linear.x = 0.0
            self.twist.angular.z = 2.0  # 增加轉向速度
        else:
            # 沿著右側牆壁行走
            if right_dist > self.right_wall_distance + 0.1:
                # 右側距離較遠，右轉
                self.twist.linear.x = 1.0  # 增加前進速度
                self.twist.angular.z = -2.5  # 增加右轉速度
            elif right_dist < self.right_wall_distance - 0.1:
                # 右側距離較近，左轉
                self.twist.linear.x = 1.0  # 增加前進速度
                self.twist.angular.z = 1.0  # 增加左轉速度
            else:
                # 保持直行
                self.twist.linear.x = 1.0  # 增加前進速度
                self.twist.angular.z = 0.0  # 保持直行

        self.cmd_vel_pub.publish(self.twist)

    def image_callback(self, data):
        current_time = time.time()

        # 如果機器人最近一次停止的時間在20秒(加上暫停的5秒)以內，則不再偵測紅色
        if current_time - self.last_stop_time < 20:
            return

        # 使用cv_bridge將ROS影像資料轉換為OpenCV格式
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # 獲取整的圖像
        height, width, _ = cv_image.shape
        total_pixels = height * width

        # 轉換為HSV色彩空間
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 定義紅色的HSV範圍
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 + mask2

        # 計算紅色區域的像素數量
        red_pixels = cv2.countNonZero(mask)

        # 計算紅色區域的佔比
        red_ratio = red_pixels / total_pixels

        # 檢測紅色區域佔比是否超過40%
        red_detected = red_ratio > 0.4

        # rospy.loginfo("Red pixel ratio: {:.2f}".format(red_ratio))  \

        if red_detected and self.front_distance < 0.4:
            rospy.loginfo("Red wall detected! Stopping for 5 seconds.")
            self.stop_movement = True
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(5)  # 停止5秒
            self.stop_movement = False
            self.last_stop_time = current_time

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        navigator = MazeNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass



