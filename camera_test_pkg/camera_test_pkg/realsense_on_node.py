import rclpy 
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import TargetPosition

import pyrealsense2 as rs
import numpy as np
import random
import cv2 as cv
import torch
import time

class realsense_on_node(Node):
    def __init__(self):
        super().__init__("realsense_on_node")

        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path = '/home/ubuntu2/UDOn_ws/src/camera_test_pkg/camera_test_pkg/weight/best.pt')

        # self.model.conf = 0.4

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        try:
            self.pipeline.start(self.config)
            self.get_logger().info("Realsense camera Node on")
        except Exception as e:
            self.get_logger().error(f"Error starting RealSense pipeline: {e}")

        self.publisher_ = self.create_publisher(TargetPosition, "target_position", 5)
        self.timer_ = self.create_timer(0.2, self.object_detect)

    def get_mid_pos(self, frame, box, depth_data, randnum):
        distance_list = []
        mid_pos = [(box[0] + box[2])//2, (box[1] + box[3])//2] #确定索引深度的中心像素位置
        min_val = min(abs(box[2] - box[0]), abs(box[3] - box[1])) #确定深度搜索范围
        #print(box,)
        for i in range(randnum):
            bias = random.randint(-min_val//4, min_val//4)
            dist = depth_data[int(mid_pos[1] + bias), int(mid_pos[0] + bias)]
            cv.circle(frame, (int(mid_pos[0] + bias), int(mid_pos[1] + bias)), 4, (255,0,0), -1)
            #print(int(mid_pos[1] + bias), int(mid_pos[0] + bias))
            if dist:
                distance_list.append(dist)
        distance_list = np.array(distance_list)
        distance_list = np.sort(distance_list)[randnum//2-randnum//4:randnum//2+randnum//4] #冒泡排序+中值滤波
        #print(distance_list, np.mean(distance_list))
        return np.mean(distance_list)

    def dectshow(self, org_img, boxs, depth_data):
        img = org_img.copy()
        for box in boxs:
            cv.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
            dist = self.get_mid_pos(org_img, box, depth_data, 24)
            cv.putText(img, box[-1] + str(dist / 10)[:4] + 'cm',(int(box[0]), int(box[1])), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv.imshow('dec_img', img)

    def object_detect(self):
        
        frame = self.pipeline.wait_for_frames()
        depth_frame = frame.get_depth_frame()
        color_frame = frame.get_color_frame()
        
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())


        results = self.model(color_image)

        boxs = results.pandas().xyxy[0].values

        detected_objects = results.pred[0]

        for det in detected_objects:
            class_id, confidence, bbox = det[5], det[4], det[:4]
            class_name = self.model.names[int(class_id)]

            if class_name == "box" and confidence >= 0.5 and bbox[2] - bbox[0] >= 250:
                    msg = TargetPosition()
                    msg.target_x1 = float(bbox[0])
                    msg.target_y1 = float(bbox[1])
                    msg.target_x2 = float(bbox[2])
                    msg.target_y2 = float(bbox[3])
                    self.publisher_.publish(msg)
                    # print(f"object: {class_name}, confidence: {confidence:.2f}, c_x1y1: {msg.target_x1}, {msg.target_y1} c_x2y2: {msg.target_x2}, {msg.target_y2}")

        self.dectshow(color_image, boxs, depth_image)
        cv.waitKey(1)

    def __del__(self):
        self.pipeline.stop()
        cv.destroyAllWindows()

def main(args = None):
    rclpy.init(args = args)
    node = realsense_on_node()

    try:
        while rclpy.ok():
            rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass

    finally:
        node.__del__()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


