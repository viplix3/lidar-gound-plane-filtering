import rospy
import logging
import numpy as np
import ros_numpy

from sensor_msgs.msg import PointCloud2

from utils.initializers_and_loader import load_params

logger = logging.getLogger(__name__)


class Publisher:
    def __init__(self, publisher_cfg: str):
        params = load_params(publisher_cfg)

        # rospy.init_node(params["node_name"])
        self.publisher = rospy.Publisher(
            params["topic_name"], PointCloud2, queue_size=params["queue_size"]
        )

    def publish(self, pcd_numpy: np.ndarray, frame_id: str):
        point_cloud2_msg = ros_numpy.point_cloud2.array_to_pointcloud2(
            pcd_numpy, rospy.Time.now(), frame_id
        )
        self.publisher.publish(point_cloud2_msg)
