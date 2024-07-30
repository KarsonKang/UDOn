import rclpy 
from rclpy.node import Node

from dynamixel_sdk_custom_interfaces.msg import TargetPosition

import cv2
import torch
import time

class camera_on_node(Node):
    def __init__(self):

        super().__init__("camera_on_node")

        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path = '/home/ubuntu2/UDOn_ws/src/camera_test_pkg/camera_test_pkg/weight/best.pt')

        self.cap = cv2.VideoCapture(0)
        
        if self.cap.isOpened():
            self.get_logger().info("Camera on")
        else:
            self.get_logger().error("Camera startup failed")
            return
        
        self.publisher_ = self.create_publisher(TargetPosition, "target_position", 5)
        
        self.timer_ = self.create_timer(0.2, self.object_detect)


    def object_detect(self):
        # self.get_logger().info("camera test")

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error("Can not capture the picture")

        result = self.model(frame)

        detected_objects = result.pred[0]

        for det in detected_objects:
            class_id, confidence, bbox = det[5], det[4], det[:4]
            class_name = self.model.names[int(class_id)]
            # x_mid = (bbox[0] + bbox[2]) / 2
            # y_mid = (bbox[1] + bbox[3]) / 2
            # width = abs(bbox[0] - bbox[2])

            if class_name == "box" and confidence >= 0.5 and bbox[2] - bbox[0] >= 250:
                msg = TargetPosition()
                msg.target_x1 = float(bbox[0])
                msg.target_y1 = float(bbox[1])
                msg.target_x2 = float(bbox[2])
                msg.target_y2 = float(bbox[3])
                self.publisher_.publish(msg)
                print(f"object: {class_name}, confidence: {confidence:.2f}, c_x1y1: {msg.target_x1}, {msg.target_y1} c_x2y2: {msg.target_x2}, {msg.target_y2}")

        annotated_frame = result.render()[0]

        cv2.imshow("Object Detection", annotated_frame)
        cv2.waitKey(1)


    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args = None):
    rclpy.init(args = args)
    node = camera_on_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
