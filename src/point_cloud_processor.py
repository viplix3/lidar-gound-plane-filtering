import rospy
import logging
import argparse
import importlib
import ros_numpy

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

    debug_info_published = False
    try:
        while not rospy.is_shutdown():
            msg = dependencies["subscriber"].get_message()
            if msg:
                msg_fields = [field.name for field in msg.fields]
                pcd = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

                if not debug_info_published:
                    logger.debug(f"Point cloud width: {msg.width}")
                    logger.debug(f"Point cloud height: {msg.height}")
                    logger.debug(f"Point cloud is_dense: {msg.is_dense}")
                    logger.debug(f"Point step: {msg.point_step}")
                    logger.debug(f"Row step: {msg.row_step}")

                    msg_field_dtypes = [field.datatype for field in msg.fields]
                    msg_fields_count = [field.count for field in msg.fields]
                    logger.debug(f"Message fields: {msg_fields}")
                    logger.debug(f"Message field datatypes: {msg_field_dtypes}")
                    logger.debug(f"Message field counts: {msg_fields_count}")

                    logger.debug(f"Point cloud data shape: {pcd.shape}")
                    logger.debug(f"Point cloud data dtype: {pcd.dtype}")

                    debug_info_published = True

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
