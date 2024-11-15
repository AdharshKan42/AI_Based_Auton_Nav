from ros import rosbag
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from PIL import Image
from pathlib import Path
import os

bridge = CvBridge()


class BagImageParser:
    def __init__(self, bag_filepath, topics=[], frequencies=[]):
        self.bag_file = rosbag.Bag(bag_filepath)
        self.topics = topics
        self.frequencies = frequencies
        if len(self.frequencies) < len(self.topics):
            self.frequencies = self.frequencies + [1] * (len(self.topics) - len(self.frequencies))


    def extract_to(self, root_dir):
        for i, t in enumerate(self.topics):
            p_str = os.path.join(root_dir, t.lstrip('/').replace('/', '_'))
            p = Path(p_str)
            p.mkdir(parents=True, exist_ok=True)
            frame_count = 0
            frequency_count = 0
            for topic, msg, stamp in self.bag_file.read_messages(topics=[t]):
                if msg._type in ['sensor_msgs/Image', 'sensor_msgs/CompressedImage']:
                    frequency_count = (frequency_count + 1) % self.frequencies[i]
                    if frequency_count == 0:
                        # Convert ROS Image message to OpenCV image
                        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
                        # Convert the OpenCV image (in BGR format) to RGB
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                        # save image
                        cv2.imwrite(os.path.join(p_str, f'{frame_count}.jpeg'), cv_image)
                        frame_count += 1
