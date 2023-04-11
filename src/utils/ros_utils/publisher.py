import rospy
import logging

from sensor_msgs.msg import PointCloud2

from utils.initializers_and_loader import load_params

logger = logging.getLogger(__name__)


class Publisher:
    def __init__(self, publisher_cfg: str):
        params = load_params(publisher_cfg)

        # rospy.init_node(params["node_name"])
        self.filtered_pcd_publisher = rospy.Publisher(
            params["filtered_pcd_topic_name"],
            PointCloud2,
            queue_size=params["queue_size"],
        )
        self.ground_plane_publisher = rospy.Publisher(
            params["ground_plane_topic_name"],
            PointCloud2,
            queue_size=params["queue_size"],
        )

    def publish(self, filtered_pcd: PointCloud2, ground_plane_pcd: PointCloud2):
        """Publishes the filtered point cloud and the ground plane

        Args:
            filtered_pcd (PointCloud2): Filtered point cloud
            ground_plane_pcd (PointCloud2): Ground plane pcd
        """
        self.filtered_pcd_publisher.publish(filtered_pcd)
        self.ground_plane_publisher.publish(ground_plane_pcd)
