import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import traceback
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class PointCloudToLaserScanBridgeNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan_bridge_node')

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_ALL, 
        )

        bridge_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, 
            history=HistoryPolicy.KEEP_ALL, 
        )

        self.create_subscription(PointCloud2, '/cloud_in', self.__pointcloud_callback, bridge_qos)
        self.create_subscription(LaserScan, '/scan', self.__scan_callback, scan_qos)

        self.laserscan_bridge_publisher = self.create_publisher(LaserScan, '/scan_reliable', bridge_qos)
        
        self._logger.info('Successfully launched!')

    def __pointcloud_callback(self, message):
        pass

    def __scan_callback(self, message):
        message.header.frame_id = 'lidar'
        self.laserscan_bridge_publisher.publish(message)


def main(args=None):
    try:
        rclpy.init(args=args)

        node = PointCloudToLaserScanBridgeNode()
        rclpy.spin(node)
        node.destroy_node()

        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(''.join(traceback.TracebackException.from_exception(e).format()))
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
