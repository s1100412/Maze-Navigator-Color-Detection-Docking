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
        self.right_wall_distance = 0.3  # 设定右侧墙壁的目标距离为0.3米
        self.stop_movement = False
        self.last_stop_time = 0
        self.front_distance = 10

    def laser_callback(self, data):
        if self.stop_movement:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            return

        # 获取激光雷达数据
        right_dist = min(min(data.ranges[0:60]), 10)  # 右侧距离
        front_dist = min(min(data.ranges[300:420]), 10)  # 前方距离
        self.front_distance = front_dist

        if front_dist < 0.5:
            # 前方有障碍物，左转
            self.twist.linear.x = 0.0
            self.twist.angular.z = 2.0  # 增加转向速度
        else:
            # 沿着右侧墙壁行走
            if right_dist > self.right_wall_distance + 0.1:
                # 右侧距离较远，右转
                self.twist.linear.x = 1.0  # 增加前进速度
                self.twist.angular.z = -2.5  # 增加右转速度
            elif right_dist < self.right_wall_distance - 0.1:
                # 右侧距离较近，左转
                self.twist.linear.x = 1.0  # 增加前进速度
                self.twist.angular.z = 1.0  # 增加左转速度
            else:
                # 保持直行
                self.twist.linear.x = 1.0  # 增加前进速度
                self.twist.angular.z = 0.0  # 保持直行

        self.cmd_vel_pub.publish(self.twist)

    def image_callback(self, data):
        current_time = time.time()

        # 如果机器人最近一次停止的时间在20秒(加上暂停的5秒)以内，则不再检测红色
        if current_time - self.last_stop_time < 20:
            return

        # 使用cv_bridge将ROS图像数据转换为OpenCV格式
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # 获取整个图像
        height, width, _ = cv_image.shape
        total_pixels = height * width

        # 转换为HSV色彩空间
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 定义红色的HSV范围
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 + mask2

        # 计算红色区域的像素数量
        red_pixels = cv2.countNonZero(mask)

        # 计算红色区域的占比
        red_ratio = red_pixels / total_pixels

        # 检测红色区域占比是否超过50%
        red_detected = red_ratio > 0.4

        # rospy.loginfo("Red pixel ratio: {:.2f}".format(red_ratio))  # 调试信息

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

# #!/usr/bin/env python3
# import rospy
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan, Image
# import time

# class MazeNavigator:
#     def __init__(self):
#         rospy.init_node('maze_navigator', anonymous=True)
#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#         self.laser_sub = rospy.Subscriber('/rrbot/laser/scan', LaserScan, self.laser_callback)
#         self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
#         self.twist = Twist()
#         self.bridge = CvBridge()
#         self.right_wall_distance = 0.3  # 设定右侧墙壁的目标距离为0.2米
#         self.stop_movement = False
#         self.last_stop_time = 0
#         self.front_distance = 10
        

#     def laser_callback(self, data):
#         if self.stop_movement:
#             self.twist.linear.x = 0.0
#             self.twist.angular.z = 0.0
#             self.cmd_vel_pub.publish(self.twist)
#             return

#         # 获取激光雷达数据
#         right_dist = min(min(data.ranges[0:60]), 10)  # 右侧距离
#         front_dist = min(min(data.ranges[300:420]), 10)  # 前方距离
#         self.front_distance = front_dist
        

#         if front_dist < 0.5:
#             # 前方有障碍物，左转
#             self.twist.linear.x = 0.0
#             self.twist.angular.z = 2.0  # 增加转向速度
#         else:
#             # 沿着右侧墙壁行走
#             if right_dist > self.right_wall_distance + 0.1:
#                 # 右侧距离较远，右转
#                 self.twist.linear.x = 1.0  # 增加前进速度
#                 self.twist.angular.z = -2.5  # 增加右转速度
#             elif right_dist < self.right_wall_distance - 0.1:
#                 # 右侧距离较近，左转
#                 self.twist.linear.x = 1.0  # 增加前进速度
#                 self.twist.angular.z = 1.0  # 增加左转速度
#             else:
#                 # 保持直行
#                 self.twist.linear.x = 1.0  # 增加前进速度
#                 self.twist.angular.z = 0.0  # 保持直行

#         self.cmd_vel_pub.publish(self.twist)

#     def image_callback(self, data):
#         current_time = time.time()

#         # 如果机器人最近一次停止的时间在20秒(加上暫停的5秒)以内，则不再检测红色
#         if current_time - self.last_stop_time < 20:
        
#             return

#         # 使用cv_bridge将ROS图像数据转换为OpenCV格式
#         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

#         # 获取整个图像
#         height, width, _ = cv_image.shape

#         # 转换为HSV色彩空间
#         hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

#         # 定义红色的HSV范围
#         lower_red1 = np.array([0, 50, 50])
#         upper_red1 = np.array([10, 255, 255])
#         mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        
#         lower_red2 = np.array([170, 50, 50])
#         upper_red2 = np.array([180, 255, 255])
#         mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

#         mask = mask1 + mask2

#         # 检测红色
#         red_detected = cv2.countNonZero(mask) > 500  # 这里可以根据需要调整阈值
        

#         if red_detected and self.front_distance < 0.2 :
#             rospy.loginfo("Red wall detected! Stopping for 5 seconds.")
#             self.stop_movement = True
#             self.twist.linear.x = 0.0
#             self.twist.angular.z = 0.0
#             self.cmd_vel_pub.publish(self.twist)
#             rospy.sleep(5)  # 停止5秒
#             self.stop_movement = False
#             self.last_stop_time = current_time

#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     try:
#         navigator = MazeNavigator()
#         navigator.run()
#     except rospy.ROSInterruptException:
#         pass



