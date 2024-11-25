import unittest
from bag_parser import BagParser


class TestBagParser(unittest.TestCase):
    def test_parse_image_success(self):
        parser = BagParser('./test_output/realsense_record.bag')
        parser.extract_color_image('./test_output/extracted_image', topics=['/realsense/color/image_raw'], frequencies=[20])
        # parser.extract_depth_image('./test_output/extracted_image', topics=['/realsense/depth/image_rect_raw'], frequencies=[5])