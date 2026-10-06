#!/usr/bin/env python3
"""Build a transient local OccupancyGrid from the live MOLA deskewed cloud.

This is an experiment-only navigation bridge: Native GICP remains the pose/map
source, while Nav2 receives a continuously refreshed 2-D grid and the original
Deskewed PointCloud2 for obstacle marking. No saved .mm map is read.
"""
import os
import math
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

class RealtimeNavGrid(Node):
    def __init__(self):
        super().__init__('mola_realtime_nav_grid')
        robot = os.environ.get('OMNIFLEET_ROBOT_ID', 'robot_113')
        self.topic = f'/{robot}/map'
        self.frame = f'{robot}/map'
        self.res = 0.10
        self.width = 500
        self.height = 500
        self.origin_x = -25.0
        self.origin_y = -25.0
        self.grid = [-1] * (self.width * self.height)
        self.sensor_frame = f'{robot}/rslidar'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending_clouds = deque(maxlen=3)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.have_pose = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, self.topic, qos)
        cloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            PointCloud2, f'/{robot}/navigation/deskewed_points', self.cloud_cb, cloud_qos)
        self.pose_sub = self.create_subscription(
            Odometry, f'/{robot}/lidar_odometry/pose', self.pose_cb, 10)
        self.update_timer = self.create_timer(0.05, self.process_pending)
        self.timer = self.create_timer(1.0, self.publish_grid)
        self.last_points = 0
        self.publish_grid()
        self.get_logger().info(f'publishing realtime grid {self.topic} from deskewed MOLA cloud')

    def cloud_cb(self, msg):
        self.pending_clouds.append(msg)

    def pose_cb(self, msg):
        frame_id = msg.header.frame_id.lstrip('/')
        if frame_id != self.frame:
            return
        self.pose_x = float(msg.pose.pose.position.x)
        self.pose_y = float(msg.pose.pose.position.y)
        self.have_pose = True

    def process_pending(self):
        if not self.pending_clouds:
            return
        msg = self.pending_clouds[0]
        frame_id = msg.header.frame_id.lstrip('/')
        if frame_id not in (self.frame, self.sensor_frame):
            self.pending_clouds.popleft()
            return
        if frame_id == self.frame:
            # MOLA's deskewed output is already expressed in map coordinates.
            # The old code nevertheless required a map->rslidar TF at the
            # sensor timestamp; that frame is not published in this setup, so
            # every cloud was silently discarded and the grid stayed unknown.
            if not self.have_pose:
                return
            sensor_x, sensor_y = self.pose_x, self.pose_y
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame, self.sensor_frame, Time.from_msg(msg.header.stamp))
            except TransformException:
                return
            sensor_x = tf.transform.translation.x
            sensor_y = tf.transform.translation.y
        self.pending_clouds.popleft()
        origin = self.cell(sensor_x, sensor_y)
        if origin is None:
            return
        updated = 0
        rays = {}
        fallback_rays = {}
        occupied = set()
        for x, y, z in point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
            x, y, z = float(x), float(y), float(z)
            dx, dy = x - sensor_x, y - sensor_y
            distance = math.hypot(dx, dy)
            if not (0.12 <= distance <= 8.0):
                continue
            cell = self.cell(x, y)
            if cell is None:
                continue
            angle_bin = int((math.atan2(dy, dx) + math.pi) * 180.0 / math.pi)
            if -0.03 <= z <= 0.5 and distance <= 6.0:
                occupied.add(cell[1] * self.width + cell[0])
                if angle_bin not in rays or distance < rays[angle_bin][0]:
                    rays[angle_bin] = (distance, cell)
            elif -2.0 <= z <= 4.0:
                if angle_bin not in fallback_rays or distance > fallback_rays[angle_bin][0]:
                    fallback_rays[angle_bin] = (distance, cell)
        for angle_bin in rays.keys() | fallback_rays.keys():
            _, cell = rays.get(angle_bin) or fallback_rays[angle_bin]
            self.mark_free_ray(origin, cell)
        for idx in occupied:
            if self.grid[idx] != 100:
                self.grid[idx] = 100
                updated += 1
        self.last_points = updated
        # Keep point-cloud rasterization off the costmap publisher path.  The
        # 1 Hz timer below is the single publication cadence; publishing the
        # full 500x500 grid for every processed cloud caused redundant
        # serialization and competed with the navigation controller.

    def cell(self, x, y):
        ix = int(math.floor((x - self.origin_x) / self.res))
        iy = int(math.floor((y - self.origin_y) / self.res))
        return (ix, iy) if 0 <= ix < self.width and 0 <= iy < self.height else None

    def mark_free_ray(self, origin, end):
        x, y = origin
        end_x, end_y = end
        dx, dy = abs(end_x - x), abs(end_y - y)
        step_x = 1 if x < end_x else -1
        step_y = 1 if y < end_y else -1
        error = dx - dy
        while True:
            idx = y * self.width + x
            # A current ray observation clears stale dynamic occupancy.
            # Current-frame hit cells are restored to occupied below.
            self.grid[idx] = 0
            if x == end_x and y == end_y:
                break
            twice = 2 * error
            if twice > -dy:
                error -= dy
                x += step_x
            if twice < dx:
                error += dx
                y += step_y

    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.info.resolution = self.res
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RealtimeNavGrid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
