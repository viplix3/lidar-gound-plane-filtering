import rospy
import logging
import importlib

from collections import deque
from sensor_msgs.msg import PointCloud2

from utils.initializers_and_loader import load_params

logger = logging.getLogger(__name__)


class Subscriber:
    def __init__(self, subscriber_cfg: str):
        params = load_params(subscriber_cfg)

        rospy.init_node(params["node_name"])
        self.subscriber = rospy.Subscriber(
            params["topic_name"], PointCloud2, self.callback
        )

        self.message_queue = deque(maxlen=params["queue_size"])

    def callback(self, msg):
        self.message_queue.append(msg)

    def get_message(self):
        return self.message_queue.popleft() if self.message_queue else None
