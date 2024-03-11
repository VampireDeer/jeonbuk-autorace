import sys
import geometry_msgs.msg
import time

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
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


        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        
        # 좌표 설정
        self.goal_poses = PoseStamped()
        self.goal_poses.header.frame_id = 'map'
        self.goal_poses.header.stamp = self.nav.get_clock().now().to_msg()

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
        self.count_intersection = 0
        self.twist_msg = self.TwistMsg()

        self.aruco_info_data = 51
        self.object_info_data = 0


        if self.stamped:
            self.twist = self.twist_msg.twist
            self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_msg.header.frame_id = self.frame_id
        else:
            self.twist = self.twist_msg

        self.lane_state_data = None
        self.slam_state_data = None


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

        self.aruco_subscriber = self.create_subscription(
            Int32MultiArray,
            self.aruco_topic,
            self.aruco_info_callback,
            10
        )

        self.detect_subscriber = self.create_subscription(
            Int32MultiArray,
            self.object_topic,
            self.object_info_callback,
            10
        )
        
        self.Twist_publisher = self.create_publisher(
            self.TwistMsg,
            '/base_controller/cmd_vel_unstamped',
            10
        )


    def lane_state_callback(self, msg):
        if msg.data:
            self.lane_state_data = msg.data[0]
            self.auto_drive()
          
    def slam_state_callback(self, msg):
        if msg.data:
            self.slam_state_data = msg.data[0]
            self.auto_drive()

    def pose_stamepd_callback(self, msg):
        self.get_logger().info(f"Recive PoseStamped message: {msg}")


    def aruco_info_callback(self, msg):
        if msg.data:
            self.aruco_info_data = msg.data[0]
            self.auto_drive()

    def object_info_callback(self, msg):
        if msg.data:
            self.object_info_data = msg.data[0]
            self.auto_drive()


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
        
    def first_intersection(self): # 첫 교차로에서 왼쪽
        self.goal_poses.pose.position.x = 1.6900200843811035
        self.goal_poses.pose.position.y = -0.6958297491073608
        self.goal_poses.pose.position.z = 0.16546630859375
        self.goal_poses.pose.orientation.x = 0.0
        self.goal_poses.pose.orientation.y = 0.0
        self.goal_poses.pose.orientation.z = -0.7980561894915864
        self.goal_poses.pose.orientation.w = 0.6025830386047796


    def second_intersection(self): # 두번째 교차로에서 위
        self.goal_poses.pose.position.x = 1.1360598802566528
        self.goal_poses.pose.position.y = -2.0756125450134277
        self.goal_poses.pose.position.z = 0.002471923828125
        self.goal_poses.pose.orientation.x = 0.0
        self.goal_poses.pose.orientation.y = 0.0
        self.goal_poses.pose.orientation.z = 0.9894361862475364
        self.goal_poses.pose.orientation.w = 0.1449690771990033

    def third_intersection(self): # 세번째 교차로에서 오른쪽
        self.goal_poses.pose.position.x = 0.3643122613430023
        self.goal_poses.pose.position.y = -1.588432788848877
        self.goal_poses.pose.position.z = 0.006378173828125
        self.goal_poses.pose.orientation.x = 0.0
        self.goal_poses.pose.orientation.y = 0.0
        self.goal_poses.pose.orientation.z = 0.5853278551954858
        self.goal_poses.pose.orientation.w = 0.8107967081409818

    def last_intersection(self): # 첫(네번)째 교차로에서 아래
        self.goal_poses.pose.position.x = 2.05649733543396
        self.goal_poses.pose.position.y = -0.435742050409317
        self.goal_poses.pose.position.z = -0.001373291015625
        self.goal_poses.pose.orientation.x = 0.0
        self.goal_poses.pose.orientation.y = 0.0
        self.goal_poses.pose.orientation.z = -0.15083122826614587
        self.goal_poses.pose.orientation.w = 0.9885595280911139


    def use_navi(self, pose):
        self.goal_poses.header.stamp = self.nav.get_clock().now().to_msg()

        time.sleep(1)
        #첫 교차로 좌회전 way_point
        pose

        print("Now Robot's Goal:\n", 
            "x = ", self.goal_poses.pose.position.x,
            "\n y = ", self.goal_poses.pose.position.y, 
            "\n z = ", self.goal_poses.pose.position.z)


        # 로봇 이동 여기가 슬렘 부분인데 잘 모르겠다. 왠 안돼는건지
        # 슬렘을 하다가 Lane Detect하면 다시 Lane을 따라가거나 하는걸 하고싶은데 안된다.
        
        if self.nav.goToPose(self.goal_poses):
            time.sleep(0.5)
            print("\nslam start\n")
            if not self.nav.isTaskComplete():
                if self.slam_state_data == 1:
                    feedback = self.nav.getFeedback()
                    if feedback:
                        print("Distance remaining: " + '{:.2f}'.format(
                            feedback.distance_remaining) + ' meters.')
                elif self.slam_state_data == 0:
                    self.nav.cancelTask()
                    print("Slam Mode cancle")
            
                rclpy.spin_once(self)

            elif self.nav.isTaskComplete():
                pass


    def auto_drive(self):
        if (self.aruco_info_data == 1) or (self.object_info_data == 1):
            
            # self.x = 0.0
            # self.y = 0.0
            # self.z = 0.0
            # self.th = 0.0
            print("detect aruco or object")
            time.sleep(3)

            print("Lane Detet Mode")
            if self.slam_state_data == 0:
            # LANE Detect Mode
                # Go_Front
                if self.lane_state_data == 1:
                    self.go_front()
                    print("GO_Front")

                # Go_Right
                elif self.lane_state_data == 2:
                    self.go_right()
                    print("Go_Right")

                # Go_Left
                elif self.lane_state_data == 3:
                    self.go_left()
                    print("Go_Left")

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
            

            # SLAM Mode
            elif self.slam_state_data == 1:
                print("Slam Mode pass")
                # pass
                
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.th = 0.0

                if self.count_intersection == 0:
                    self.use_navi(self.first_intersection)
                    self.count_intersection = 1
                elif self.count_intersection == 1:
                    self.use_navi(self.second_intersection)
                    self.count_intersection = 2
                elif self.count_intersection == 2:
                    self.use_navi(self.third_intersection)
                    self.count_intersection = 3
                elif self.count_intersection == 3:
                    self.use_navi(self.last_intersection)
                    self.count_intersection = 4
                else:
                    print("5 intersection is nothing")    
                    pass
            else:
                print("Not Lane detect, Slam")

            if self.stamped:
                self.twist_msg.header.stamp = self.get_clock().now().to_msg()

            self.twist.linear.x = self.x * self.speed
            self.twist.linear.y = self.y * self.speed
            self.twist.linear.z = self.z * self.speed
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = self.th * self.turn
            self.Twist_publisher.publish(self.twist_msg)



        else:
            print("Lane Detet Mode")
            if self.slam_state_data == 0:
            # LANE Detect Mode
                # Go_Front
                if self.lane_state_data == 1:
                    self.go_front()
                    print("GO_Front")

                # Go_Right
                elif self.lane_state_data == 2:
                    self.go_right()
                    print("Go_Right")

                # Go_Left
                elif self.lane_state_data == 3:
                    self.go_left()
                    print("Go_Left")

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
                print("Lane Detet Mode acting")
            

            # SLAM Mode
            elif self.slam_state_data == 1:
                print("Slam Mode pass")
                # pass
                
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.th = 0.0

                if self.count_intersection == 0:
                    self.use_navi(self.first_intersection)
                    self.count_intersection = 1
                elif self.count_intersection == 1:
                    self.use_navi(self.second_intersection)
                    self.count_intersection = 2
                elif self.count_intersection == 2:
                    self.use_navi(self.third_intersection)
                    self.count_intersection = 3
                elif self.count_intersection == 3:
                    self.use_navi(self.last_intersection)
                    self.count_intersection = 4
                else:
                    print("5 intersection is nothing")    
                    pass
            else:
                print("Not Lane detect, Slam")

            if self.stamped:
                self.twist_msg.header.stamp = self.get_clock().now().to_msg()

            self.twist.linear.x = self.x * self.speed
            self.twist.linear.y = self.y * self.speed
            self.twist.linear.z = self.z * self.speed
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = self.th * self.turn
            self.Twist_publisher.publish(self.twist_msg)


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