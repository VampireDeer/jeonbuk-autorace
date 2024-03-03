import sys
import geometry_msgs.msg
import time

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Pose
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseWithCovarianceStamped


from sensor_msgs.msg import LaserScan

from rclpy.node import Node
import threading
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32

from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult

from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose

class Control(Node):
    def __init__(self):
        super().__init__('control')


        # self.nav = BasicNavigator()
        # self.nav.waitUntilNav2Active()
        
        # 좌표 설정
        self.goal_poses = PoseStamped()
        self.goal_poses.header.frame_id = 'map'
        # self.goal_poses.header.stamp = self.nav.get_clock().now().to_msg()

        # parameters
        self.stamped = self.declare_parameter('stamped', False).value
        self.frame_id = self.declare_parameter('frame_id', '').value


        self.lane_topic = self.declare_parameter('lane_topic', "/lane_state").value
        self.slam_topic = self.declare_parameter('slam_topic', '/slam_state').value

        self.aruco_topic = self.declare_parameter('aruco_topic', '/detect_aruco_num').value
        self.object_topic = self.declare_parameter('object_topic', '/detect_object').value


        if not self.stamped and self.frame_id:
            raise Exception("'frame_id' can only be set when 'stamped' is True")

        if self.stamped:
            self.TwistMsg = geometry_msgs.msg.TwistStamped
        else:
            self.TwistMsg = geometry_msgs.msg.Twist

        self.speed = 0.1
        self.turn = 0.5
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0.0

        self.twist_msg = self.TwistMsg()

        if self.stamped:
            self.twist = self.twist_msg.twist
            self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_msg.header.frame_id = self.frame_id
        else:
            self.twist = self.twist_msg

        self.lane_state_data = None
        self.slam_state_data = None

        self.intersection = 0

        self.lane_state_subscription = self.create_subscription(
            Int32MultiArray,
            self.lane_topic,
            self.lane_state_callback,
            10
        )

        self.slam_state_subscription = self.create_subscription(
            Int32MultiArray,
            self.slam_topic,
            self.slam_state_callback,
            10
        )


        # self.aruco_subscriber = self.create_subscription(
        #     Int32MultiArray,
        #     self.aruco_topic,
        #     self.aruco_info_callback(),
        #     10
        # )

        # self.detect_subscriber = self.create_subscription(
        #     Int32MultiArray,
        #     self.object_topic,
        #     self.object_info_callback(),
        #     10
        # )
        
        self.position_subscriber = self.create_subscription(
            PoseStamped,
            "/pose_state",
            self.pose_state_callback,
            10
        )

        self.Twist_publisher = self.create_publisher(
            self.TwistMsg,
            '/base_controller/cmd_vel_unstamped',
            10
        )

        self.gorani = 0
        self.checkcheck = 0

    def lane_state_callback(self, msg):
        if msg.data:
            self.lane_state_data = msg.data[0]
            self.auto_drive()
          
    def slam_state_callback(self, msg):
        if msg.data:
            self.slam_state_data = msg.data[0]
            self.auto_drive()

    def pose_state_callback(self, msg):
        if msg.pose:
            self.position_state_data = msg.pose.position
            self.orientation_state_data = msg.pose.orientation
            self.auto_drive()

    # def aruco_info_callback(self, msg):
    #     if msg.data:
    #         self.aruco_info_data = msg.data[0]
    #         self.auto_drive()
          
    # def object_info_callback(self, msg):
    #     if msg.data:
    #         self.object_info_data = msg.data[0]
    #         self.auto_drive()
    def moter_time(self, x, speed, theta, time_action):
        start_time = time.time()
                # print(run_time)
        while True:
            run_time = time.time() - start_time
            self.twist.linear.x = x * speed
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = theta * speed

            # 속도와 방향에 대한 정보를 pub
            self.Twist_publisher.publish(self.twist_msg)
            # pub.publish(twist_msg)
            # print(run_time)

            # 로봇 구동
            if self.stamped:
                self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            
            # while문 동작시간이 time_action 시간보다 넘어가면 종료
            # + 0.5는 ros로 송수신하면서 실제로 모터가 작동되는 시간이 time_action보다 줄어들기 때문에, 이를 보정하기 위해 임의로 입력한 수치
            if (run_time > time_action +0.5):
                break
            else:
                pass







    def moter_state(self):
        if self.stamped:
            self.twist_msg.header.stamp = self.get_clock().now().to_msg()


        self.twist.linear.x = self.x * self.speed
        self.twist.linear.y = self.y * self.speed
        self.twist.linear.z = self.z * self.speed
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = self.th * self.turn
        self.Twist_publisher.publish(self.twist_msg)


    def go_front(self):
        self.x = 1.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0

    def go_right(self):
        self.x = 0.2
        self.y = 0.0
        self.z = 0.0
        self.th = -0.3

    def go_left(self):
        self.x = 0.2
        self.y = 0.0
        self.z = 0.0
        self.th = 0.3

    def go_stop(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0

    def turn_right(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = -0.3

    def turn_left(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.3
        
    def go_front_time(self):
        start_time = time.time()
        print("start_time_fuck")
        self.run_time  = time.time() - start_time


        self.x = 1.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        print("12341234")

        if self.run_time > 0.5:
            print("fuck out")
            return

            
    def go_right_time(self):
        start_time = time.time()

        run_time = time.time() - start_time

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = -1.0

        if run_time > 1:
            return

    def go_left_time(self):
        start_time = time.time()

        run_time  = time.time() - start_time

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 1.0

        if run_time > 1:
            return


    def auto_drive(self):
        print("Lane Detet Mode")
        if self.slam_state_data == 0:
            self.gorani = 0
            print("gorani change 1 to 0")
        # LANE Detect Mode
            # Go_Front
            if self.lane_state_data == 1:
                self.go_front()

            # Go_Right
            elif self.lane_state_data == 2:
                self.go_right()

            # Go_Left
            elif self.lane_state_data == 3:
                self.go_left()

            else:
                pass
            
            if self.stamped:
                self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.speed = 0.1
            self.twist.linear.x = self.x * self.speed *(2/3)
            self.twist.linear.y = self.y * self.speed *(2/3)
            self.twist.linear.z = self.z * self.speed *(2/3)
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = self.th * self.turn
            self.Twist_publisher.publish(self.twist_msg)
            # self.pub.publish(self.twist_msg)
            print("Lane Detet Mode acting")
        

        # Hard coding
        elif self.slam_state_data == 1:
            print("Hard coding start")
            print(self.checkcheck)
            print(self.intersection)
            self.checkcheck += 1
            # if self.intersection == 0:
            #     self.go_front_time()
            #     self.moter_state()
            #     print("asdfwefa")
            #     self.go_left_time()
            #     self.moter_state()
            #     self.moter_time(0.0, 1, 0.3, 8)
            #     self.moter_time(0.0, 1, 0.3, 8)

            #     self.intersection += 1
            #     print(self.intersection)
            if self.gorani == 0:

                if self.intersection == 0:
                    inter_1st = time.time()
                    self.moter_time(0.3, 0.2, 0.0, 5)
                    self.moter_time(0.0, 1, 0.3, 8)
                    self.moter_time(0.0, 0.0, 0.0, 8)
                   
                    inter_1st_run = time.time() - inter_1st
                    print("1st inter secction running", inter_1st_run, "sec")
                    if inter_1st_run > 25:
                        pass
                    self.gorani = 1
                    self.intersection = 1
                   

                    # now_time = time.time()
                    # if time.time() - now_time > 10:
                    #     self.intersection = 1
                    # else:
                    #     self.intersection = 0
                    #     print("delay....")

                    print(self.intersection)
                    
                        
                elif self.intersection == 1:
                    inter_1st = time.time()
                    self.moter_time(0.3, 0.2, 0.0, 5)
                    print("debug1")
                    self.moter_time(0.0, 1, -0.3, 8)
                    print("debug2")
                    self.moter_time(0.0, 0.0, 0.0, 8)
                    print("debug3")
                    inter_1st_run = time.time() - inter_1st
                    print("1st inter secction running", inter_1st_run, "sec")
                    if inter_1st_run > 25:
                        pass
                    self.gorani = 1
                    self.intersection = 2
                    
                    

                elif self.intersection == 2:

                    self.moter_time(0.3, 0.2, 0.0, 5)
                    print("debug4")
                    self.moter_time(0.0, 1, -0.3, 8)
                    print("debug5")
                    self.moter_time(0.0, 0.0, 0.0, 8)
                    print("debug6")
                    self.gorani = 1
                    self.intersection = 3

                    

                elif self.intersection == 3:

                    self.moter_time(0.3, 0.2, 0.0, 7)
                    self.gorani = 1
                    self.intersection = 4
                    
                
                else:
                    self.moter_time(1.0, 1, 0.0, 8)

                    return

            elif self.gorani == 1:
                self.moter_time(0.0, 0.0, 0.0, 0)
                # print("gorani do not change")

                return
        else:
            pass


        # if self.stamped:
        #     self.twist_msg.header.stamp = self.get_clock().now().to_msg()


        # self.twist.linear.x = self.x * self.speed
        # self.twist.linear.y = self.y * self.speed
        # self.twist.linear.z = self.z * self.speed
        # self.twist.angular.x = 0.0
        # self.twist.angular.y = 0.0
        # self.twist.angular.z = self.th * self.turn
        # self.Twist_publisher.publish(self.twist_msg)
        # # self.pub.publish(self.twist_msg)

        #     # if self.stamped:
            #     self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            # self.speed = 0.1
            # self.twist.linear.x = self.x * self.speed *(2/3)
            # self.twist.linear.y = self.y * self.speed *(2/3)
            # self.twist.linear.z = self.z * self.speed *(2/3)
            # self.twist.angular.x = 0.0
            # self.twist.angular.y = 0.0
            # self.twist.angular.z = self.th * self.turn
            # self.Twist_publisher.publish(self.twist_msg)
            # # self.pub.publish(self.twist_msg)
            # print("Lane Detet Mode acting")
        

def main(args=None):
    rclpy.init(args=args)

    control = Control()

    # rclpy.spin(control)
    try:
        rclpy.spin(control)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()