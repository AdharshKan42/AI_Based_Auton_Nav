#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import Point

class YOLOInferenceNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolo_inference_node', anonymous=True)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Load YOLOv8 model
        self.model = YOLO("/home/rover/Documents/try_2/AI_Based_Auton_Nav/catkin_ws/src/utm_to_robot/scripts/best.pt")  # Replace with your path to yolov8 model

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/realsense/color/image_raw', Image, self.image_callback)
        self.center_pub = rospy.Publisher('/green_bounding_box_center', Point, queue_size=10)


    def image_callback(self, msg):
        try:
            # Convert the ROS image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform inference on the image using YOLOv8
            results = self.model(cv_image)

            # Get the bounding boxes, class labels, and confidence scores
            boxes = results[0].boxes  # Get the bounding box information
            classes = results[0].names  # Get the class names (labels)
            conf = results[0].probs  # Get the confidence scores

            # Print or store the detected bounding boxes, class labels, and confidence
            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()  # Bounding box coordinates
                class_id = int(box.cls[0].cpu().numpy())  # Class index
                class_name = classes[class_id]  # Get the class name from class index
                confidence = conf[i].cpu().numpy()  # Confidence score

                if class_name == "green_object" and confidence > 0.5:  # Check if it's a green object with confidence
                    # Calculate the center of the bounding box
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    # Publish the center to the topic
                    center_point = Point(center_x, center_y, 0)
                    self.center_pub.publish(center_point)

                # Log or process the bounding box information
                rospy.loginfo(f"Detected {class_name} with confidence {confidence:.2f} at "
                            f"({x1}, {y1}) to ({x2}, {y2})")

            # Show the image with the detections
            # annotated_frame = results[0].plot()  # Get the annotated image with boxes

            # # Show the image with bounding boxes
            # cv2.imshow("YOLOv8 Inference", annotated_frame)
            # cv2.waitKey(1)  # Display the image for a short time

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def spin(self):
        # Keeps the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        node = YOLOInferenceNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
