import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from rclpy.duration import Duration
from cv_bridge import CvBridge, CvBridgeError

import cv2
import sys

import numpy as np

from ultralytics import YOLO



class ALL_Detect(Node):
    def __init__(self):
        super().__init__('all_detect')
        
        self.ARUCO_DICT= {
                    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
                    }
        

        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.CvImage,
            10 
            )

        self.aruco_publisher = self.create_publisher(
            Int32MultiArray,
            '/detect_aruco_num',
            10
            )  
 
        self.detect_publisher = self.create_publisher(
            Int32MultiArray,
            '/detect_object',
            10
        )

        self.bridge = CvBridge()

        # 사용할 파라미터를 불러온다    
        self.declare_parameter('rgb_topic', '/image_raw/compressed')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter("device", "cpu") 
        # 파라미터의 값을 저장한다
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.conf = self.get_parameter('conf').value
        self.device = self.get_parameter('device').value


        # 학습한 YOLO 모델 불러오기
        self.model = YOLO('second_human.pt')

    def CvImage(self, msg):

        ### Image 
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("CvBridge is not working, check plz")

        ### Aruco
        def aruco_display(self, corners, ids, rejected, image):
            
            if len(corners) > 0:
                # ArUco 마커의 ID 목록을 1차원 배열로 변환
                ids = ids.flatten()

                for (markerCorner, markerID) in zip(corners, ids):
                    # 마커 모서리 추출
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    # 마커 모서리 그리기
                    cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                    # 마커 중심점 그리기
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                    # 마커 ID 번호 쓰기
                    cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            return image

        def aruco_detect(self, camera, video, aruco_type):
            # ArUco 타입이 맞지 않으면 종료하기
            if self.ARUCO_DICT.get(aruco_type, None) is None :
                print(f"ArUCo tag type '{aruco_type}' is not supproted")
                sys.exit(0)

            aruco_dict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[aruco_type])
            aruco_params = cv2.aruco.DetectorParameters_create()

            h, w, _ = self.image.shape

            width = 100
            height = int(width * (h / w))
            frame = cv2.resize(self.image, (width, height), interpolation=cv2.INTER_CUBIC)
            # corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict,
            #                                                   parameters=aruco_params)
            corners, ids, rejected = cv2.aruco.detectMarkers(self.image, aruco_dict,
                                                              parameters=aruco_params)

            # 마커가 인식된 경우에만 Print 하기
            if ids is not None and len(ids) > 0:
                for markerID in ids :
                    print("Detect ArUco marker ID: {}".format(markerID))

                    # detect_aruco_num Topic으로 보내기
                    msg = Int32MultiArray()
                    msg.data = [int(markerID)]
                    self.aruco_publisher.publish(msg)

            # 마커 정보 화면에 그리기
            # detected_markers = aruco_display(self, corners, ids, rejected, frame)
            detected_markers = aruco_display(self, corners, ids, rejected, self.image)
            return detected_markers
        
        ################################################################################
        ### YOLO
        results = self.model.predict(self.image, conf=self.conf, device=self.device) #detect
        result = results[0].boxes.data.cpu().numpy()
        objects = result[result[:,-1] == 0]

        # object 인식하면 1로 인식 못하면 0으로 topic 발행
        if len(objects) > 0:
            # tmp = np.reshape(objects, -1) #1차원 array로 만들기
            msg = Int32MultiArray()
            # tmp = tmp.astype(int).tolist()
            # msg.data = tmp
            msg.data = [1]
            self.detect_publisher.publish(msg) #/object_detect으로 퍼블리시
        else:
            msg = Int32MultiArray()
            msg.data = [0]
            self.detect_publisher.publish(msg)
        
        id = 0
        for row in objects:
            id += 1
            x1, y1, x2, y2, conf, cls = row
            cx = int(x1 + x2) // 2
            cy = int(y1 + y2) // 2
            center = (cx, cy)

            cv2.rectangle(self.image, (int(x1), int(y1)), (int(x2), int(y2)), (0,255, 0), 1)
            cv2.circle(self.image, center, 3, (0, 0, 255), -1)
            cv2.putText(self.image, "object " + str(id), (int(x1), int(y1)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)




        last_iamge = aruco_detect(self, camera=True, video=None, aruco_type="DICT_4X4_50")

        cv2.imshow("image_raw/compressed", last_iamge)
        cv2.waitKey(1)



def main(args=None):

    rclpy.init(args=args)
    detect_all = ALL_Detect()
    rclpy.spin(detect_all)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    


