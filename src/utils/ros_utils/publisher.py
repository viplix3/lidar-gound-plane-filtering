import rospy
import logging

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

    def publish(self, msg: PointCloud2):
        self.publisher.publish(msg)
