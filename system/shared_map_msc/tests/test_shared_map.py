import unittest
from rclpy.serialization import serialize_message
from nav_msgs.msg import OccupancyGrid
from omnifleet_msc.shared_map import encode_grid,decode_grid
class SharedMapTests(unittest.TestCase):
 def test_exact_cells_origin_resolution_and_checksum(self):
  m=OccupancyGrid();m.header.frame_id='map';m.info.width=3;m.info.height=2;m.info.resolution=.1
  m.info.origin.position.x=-3.4;m.info.origin.orientation.w=1.;m.data=[0,100,-1,23,99,0]
  encoded=encode_grid(m);decoded=decode_grid(encoded)
  self.assertEqual(decoded.header.frame_id,'fleet_map');self.assertEqual(list(decoded.data),list(m.data))
  self.assertEqual(serialize_message(decoded),serialize_message(m))
  encoded['sha256']='invalid'
  with self.assertRaisesRegex(ValueError,'checksum'):decode_grid(encoded)
 def test_corrupt_compressed_data_rejected(self):
  with self.assertRaises(Exception):decode_grid({'cdr_zlib':'AAAA','sha256':'x'})
