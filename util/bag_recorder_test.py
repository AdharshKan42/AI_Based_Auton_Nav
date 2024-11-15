import unittest
from bag_recorder import BagRecorderConfig, BagRecorder

class TestBagRecorder(unittest.TestCase):
    def test_record_bag_file(self):
        test_recorder_config = BagRecorderConfig(
            topics=['/front/left/image_raw'], 
            duration=20, 
            output_file='./test_output/test_bag_record.bag'
        )
        recorder = BagRecorder(test_recorder_config)
        recorder.start_recording()