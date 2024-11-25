from ros import rosbag
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from PIL import Image
from pathlib import Path
import os

bridge = CvBridge()


class BagParser:
    def __init__(self, bag_filepath):
        self.bag_file = rosbag.Bag(bag_filepath)


    def _extract_image(self, msg_processor, root_dir, topics=[], frequencies=[]):
        if len(frequencies) < len(topics):
            frequencies += [1] * (len(topics) - len(frequencies))
        for i, t in enumerate(topics):
            save_dir = os.path.join(root_dir, t.lstrip('/').replace('/', '_'))
            p = Path(save_dir)
            p.mkdir(parents=True, exist_ok=True)
            frequency_count = 0
            for topic, msg, stamp in self.bag_file.read_messages(topics=[t]):
                if msg._type in ['sensor_msgs/Image', 'sensor_msgs/CompressedImage']:
                    frequency_count = (frequency_count + 1) % frequencies[i]
                    if frequency_count == 0:
                        msg_processor(msg, save_dir, stamp)



    def extract_color_image(self, root_dir, topics=[], frequencies=[]):
        def _color_image_processor(msg, save_dir, stamp):
            # Convert ROS Image message to OpenCV image
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Convert the OpenCV image (in BGR format) to RGB
            # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # save image
            cv2.imwrite(os.path.join(save_dir, f'{stamp}.jpeg'), cv_image)

        self._extract_image(_color_image_processor, root_dir, topics=topics, frequencies=frequencies)
                        


    def extract_depth_image(self, root_dir, topics=[], frequencies=[]):
        def _depth_image_processor(msg, save_dir, stamp):
            # Convert ROS Image message to OpenCV image
            depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            normalized_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            # save image
            cv2.imwrite(os.path.join(save_dir, f'{stamp}.png'), normalized_image)
            
        self._extract_image(_depth_image_processor, root_dir, topics=topics, frequencies=frequencies)
                        