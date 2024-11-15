import unittest
from bag_parser import BagImageParser


class TestBagParser(unittest.TestCase):
    def test_parse_image_success(self):
        parser = BagImageParser('./test_output/test_bag_record.bag', topics=['/front/left/image_raw'], frequencies=[5])
        parser.extract_to('./test_output/extracted_image')