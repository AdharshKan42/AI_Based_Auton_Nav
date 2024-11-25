import unittest
from bag_recorder import BagRecorderConfig, BagRecorder

class TestBagRecorder(unittest.TestCase):
    def test_record_bag_file(self):
        test_recorder_config = BagRecorderConfig(
            topics=['/realsense/color/image_raw', '/realsense/depth/image_rect_raw', '/realsense/depth/color/points'], 
            duration=60,
            output_file='./test_output/realsense_record.bag'
        )
        recorder = BagRecorder(test_recorder_config)
        recorder.start_recording()