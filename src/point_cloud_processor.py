import rospy
import logging
import argparse
import importlib

from pathlib import Path
from typing import Dict

from utils.ros_utils import Subscriber, Publisher


def parse_args():
    parser = argparse.ArgumentParser(description="Point Cloud Processor")
    parser.add_argument(
        "--log_level",
        type=str,
        required=False,
        default="debug",
        help="Log level (debug, info, warning, error, critical)",
    )

    parser.add_argument(
        "--static_tf_cfg",
        type=str,
        required=False,
        default="configs/static_tf_broadcaster.yaml",
        help="Path to transform configuration file",
    )

    parser.add_argument(
        "--publisher_cfg",
        type=str,
        required=False,
        default="configs/publisher.yaml",
        help="Path to the ros publisher configuration file",
    )

    parser.add_argument(
        "--subscriber_cfg",
        type=str,
        required=False,
        default="configs/subscriber.yaml",
        help="Path to the ros subscriber configuration file",
    )

    return parser.parse_args()


def initialize_dependencies(args):
    subscriber = Subscriber(args.subscriber_cfg)
    return {"subscriber": subscriber}


def filter_point_cloud(args: argparse.Namespace, dependencies: Dict):
    logger.info("Starting point cloud filtering")
    try:
        while not rospy.is_shutdown():
            msg = dependencies["subscriber"].get_message()
            if msg:
                logger.debug("Message description: %s", msg._connection_header)
                logger.debug("Message info: %s", msg._full_text)
                logger.debug("Message header: %s", msg.header)
                logger.debug("Message height: %s", msg.height)
                logger.debug("Message width: %s", msg.width)
                logger.debug("Message fields: %s", msg.fields)
                logger.debug("Message is_bigendian: %s", msg.is_bigendian)
                logger.debug("Message point_step: %s", msg.point_step)
                logger.debug("Message row_step: %s", msg.row_step)
                logger.debug("Message data: %s", msg.data)
                logger.debug("Message is_dense: %s", msg.is_dense)

                for field in msg.fields:
                    logger.info(field.name)

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        logger.info("Shutting down")


if __name__ == "__main__":
    args = parse_args()

    logfile_dir = Path(__file__).parent.parent / "logs"
    logfile_dir.mkdir(parents=True, exist_ok=True)

    dependencies = initialize_dependencies(args)

    # https://github.com/ros/ros_comm/issues/1384
    importlib.reload(logging)
    logging.basicConfig(
        filename=logfile_dir / "app.log",
        filemode="w",
        level=args.log_level.upper()
        if isinstance(args.log_level, str)
        else args.log_level,
        format="%(asctime)s [%(levelname)8s] [%(filename)s:%(lineno)d]: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logger = logging.getLogger(__name__)

    filter_point_cloud(args, dependencies)
