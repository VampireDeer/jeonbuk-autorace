
import rclpy

import time
from rclpy.node import Node
import threading
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import CompressedImage
from rclpy.duration import Duration
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseWithCovarianceStamped

# # 관심영역 설정하기
# def 

class LaneDetect(Node):
    def __init__(self):
        super().__init__('lane_detect')

        self.declare_parameter('rgb_topic', '/image_raw/compressed')

        self.rgb_topic = self.get_parameter('rgb_topic').value

        # 노란선 
        self.subscription = self.create_subscription(
            CompressedImage, 
            self.rgb_topic, 
            self.y_CvImage, 
            10 ) 


        # 흰 선
        # self.subscription = self.create_subscription(
        #     CompressedImage, 
        #     self.rgb_topic, 
        #     self.w_CvImage, 
        #     10 )

        # state
        self.lane_state_publisher = self.create_publisher(
            Int32MultiArray,
            '/lane_state',
            10
        )

        # slam모드 
        self.slam_state_publisher = self.create_publisher(
            Int32MultiArray,
            '/slam_state',
            10
        )


        self.bridge = CvBridge()
        
        self.get_logger().info("Ready to detecting!")
            
        # 상태 기본 설정 값
        self.state = 0
        self.mode = 0
        
    def state_topic(self, number):
        self.state = number
        msg_state = Int32MultiArray()
        msg_state.data = [number]
        self.lane_state_publisher.publish(msg_state)

    def Lane_detect_mode(self):
        self.mode = 0
        msg_mode = Int32MultiArray()
        msg_mode.data = [0]
        self.slam_state_publisher.publish(msg_mode)

    def Slam_mode(self):
        self.mode = 1
        msg_mode = Int32MultiArray()
        msg_mode.data = [1]
        self.slam_state_publisher.publish(msg_mode)


    def y_CvImage(self,msg):

        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("Oops, CvBridge isn't working!")
        
        # 영상에서 특정 영역의 색상만 추출시킨다.
        def bird(img):
            imshape = img.shape

            # width = imshape[1]
            height = imshape[0]

            pts1 = np.float32([[60, 200],[60, height],[340,200],[340, height]]) 

            # 좌표의 이동점
            pts2 = np.float32([[10,10],[10,1000],[1000,10],[1000,1000]])

            M = cv2.getPerspectiveTransform(pts1, pts2)

            img = cv2.warpPerspective(img, M, (1100,1000))

            b, g, r = cv2.split(img)

            new_img = cv2.merge((b,g,r))

            return new_img

        new_img = bird(img)


        def remake(img):
            dst1 = cv2.inRange(img, (0,160,160), (150, 255, 255))
            # dst1 = cv2.resize(dst1, (960,540))
            return dst1
    
        edges = remake(new_img)

        def region_of_interest(img, vertices):

            mask = np.zeros_like(img)

            if len(img.shape) > 2:
                channel_count = img.shape[2]
                ignore_mask_color = (255,) * channel_count
            else:
                ignore_mask_color = 255

            cv2.fillPoly(mask, vertices, ignore_mask_color)

            masked_image = cv2.bitwise_and(img, mask)
            
            return masked_image

        new_imshape = new_img.shape

        height = new_imshape[0]
        width = new_imshape[1]

        vertices = np.array([[(width/4, height),
                                (width/4,0),
                                (width*3/4,0),
                                (width*3/4, height)]], dtype=np.int32)

        mask = region_of_interest(edges, vertices)


######
# 바꾼 번호
# state
# 0 = [default state] 
# 1 = Front
# 2 = Right
# 3 = Left
# 4 = back
# 5 = stop

# mode 
# 0 = lane_detect
# 1 = slam_on


        def draw_lines(img, lines, color=[125, 125, 255], thickness=5):
            avg_x = []
            avg_y = []

            if lines is not None:
                for line in lines:
                    for x1, y1, x2, y2 in line:
                        cv2.line(img, (x1, y1), (x2, y2), color, thickness)
                        avg_x_val = (x1+x2)/2
                        avg_y_val = (y1+y2)/2
                        avg_x.append(avg_x_val)
                        avg_y.append(avg_y_val)
            
                total_avg_x = sum(avg_x) / len(avg_x) 
                total_avg_y = sum(avg_y) / len(avg_y) 

                self.Lane_detect_mode()
                print("Lane_detect Start")

##################### Drive, Curve Algorithm #####################

                # state = 1, Front
                if (width/2 - 150) <= total_avg_x <= (width/2 + 150):
                    print("average x : ", total_avg_x)
                    print(" GO_FRONT_moter_output up ! ")
                    self.state_topic(1)

                # state = 2, Right 
                elif total_avg_x > (width/2 + 150):
                    print("average x : ", total_avg_x)
                    print(" Go_Right moter_output up ! ")
                    self.state_topic(2)

                # state = 3, Left  
                elif total_avg_x < (width/2 - 150):
                    print("average x : ", total_avg_x)
                    print(" Go_Left moter_output up ! ")
                    self.state_topic(3)

                # state = 0, detect noting state
                else :
                    print("Detect Nothing")
                    self.state_topic(0)
                # state = 4, Back   back할 일이 없을듯?
                # if total_avg_x < (width/2 - 150):
                #     print("average x : ", total_avg_x)
                #     print(" Go_back moter_output up ! ")
                #     state_topic(4)
##################### Mode Change Algorizm #######################
            else :  # 920 
                self.Slam_mode()
                self.state_topic(0)
                print("SLAM start")

                

        def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
            lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
            line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
            draw_lines(line_img, lines)

            return line_img

        rho = 2
        theta = np.pi/180
        threshold = 90
        min_line_len = 120
        max_line_gap = 150

        lines = hough_lines(mask, rho, theta, threshold, min_line_len, max_line_gap)


        # def weighted_img(img, initial_img, a=0.8, b=1., c=0.):
        #     return cv2.addWeighted(initial_img, a, img, b,c)

        # lines_edges = weighted_img(lines,new_img, a=0.8, b=1., c=0.)
        cv2.imshow('w_image_raw/compressed', lines)
        cv2.imshow('original', img)
        cv2.waitKey(1)


                
################################################################################

    # def w_CvImage(self,msg):

    #     try:
    #         img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    #     except CvBridgeError:
    #         print("Oops, CvBridge isn't working!")
        
    #     # 영상에서 특정 영역의 색상만 추출시킨다.
    #     def bird(img):
    #         imshape = img.shape

    #         width = imshape[1] # ㅡ
    #         height = imshape[0] #  |

    #         # pts1 = np.float32([[120, 200],[60, height],[280,200],[340, height]]) 
    #         # pts1 = np.float32([[0 ,0],[0, height],[width,0],[width, height]])
    #         pts1 = np.float32([[0 , height/2],[0, height],[width,height/2],[width, height]])
            

    #         # 좌표의 이동점
    #         pts2 = np.float32([[10,10],[10,1000],[1000,10],[1000,1000]])

    #         M = cv2.getPerspectiveTransform(pts1, pts2)

    #         img = cv2.warpPerspective(img, M, (1100,1000))

    #         b, g, r = cv2.split(img)

    #         new_img = cv2.merge((b,g,r))

    #         return new_img

    #     new_img = bird(img)


    #     def remake(img):
    #         dst1 = cv2.inRange(img, (200,200,200), (255, 255, 255))
    #         # dst1 = cv2.resize(dst1, (960,540))
    #         return dst1
    
    #     edges = remake(new_img)

    #     def region_of_interest(img, vertices):

    #         mask = np.zeros_like(img)

    #         if len(img.shape) > 2:
    #             channel_count = img.shape[2]
    #             ignore_mask_color = (255,) * channel_count
    #         else:
    #             ignore_mask_color = 255

    #         cv2.fillPoly(mask, vertices, ignore_mask_color)

    #         masked_image = cv2.bitwise_and(img, mask)
            
    #         return masked_image

    #     new_imshape = new_img.shape

    #     height = new_imshape[0]
    #     width = new_imshape[1]

    #     vertices = np.array([[(width/4, height),
    #                             (width/4,0),
    #                             (width*3/4,0),
    #                             (width*3/4, height)]], dtype=np.int32)

    #     mask = region_of_interest(edges, vertices)

    #     def draw_lines(img, lines, color=[255, 255, 255], thickness=5):
    #         if lines is not None:
    #             # print("!! SLAM Start!!")
    #             for line in lines:
                    
    #                 for x1, y1, x2, y2 in line:
    #                     cv2.line(img, (x1, y1), (x2, y2), color, thickness)
        
    #     def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    #         lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    #         line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    #         draw_lines(line_img, lines)

    #         return line_img

    #     rho = 2
    #     theta = np.pi/180
    #     threshold = 90
    #     min_line_len = 120
    #     max_line_gap = 150

    #     lines = hough_lines(mask, rho, theta, threshold, min_line_len, max_line_gap)


    #     # def weighted_img(img, initial_img, a=0.8, b=1., c=0.):
    #     #     return cv2.addWeighted(initial_img, a, img, b,c)

    #     # lines_edges = weighted_img(lines,new_img, a=0.8, b=1., c=0.)
    #     cv2.imshow('w_image_raw/compressed', lines)
    #     cv2.imshow('original', img)
    #     cv2.waitKey(1)


            

def main(args=None):
    # Gazebo Setup!
    rclpy.init(args=args)
    
    lane_detect = LaneDetect()
    rclpy.spin(lane_detect)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    

