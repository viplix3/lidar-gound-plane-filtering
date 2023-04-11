import rospy
import logging
import argparse
import importlib
import ros_numpy
import numpy as np

from pathlib import Path
from typing import Dict, List
from sensor_msgs.msg import PointCloud2, PointField

from utils.pre_processing import pre_processor
from utils.ros_utils import Subscriber, Publisher
from utils.initializers_and_loader import load_params
from filtering_algos.noise_fiiltering import noise_filter
from filtering_algos.ground_plane_filtering import ground_plane_filter


def parse_args():
    parser = argparse.ArgumentParser(description="Point Cloud Processor")

    parser.add_argument(
        "--node_name",
        type=str,
        required=False,
        default="point_cloud_processor",
        help="Name of the ROS node",
    )

    parser.add_argument(
        "--log_level",
        type=str,
        required=False,
        default="debug",
        help="Log level (debug, info, warning, error, critical)",
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

    parser.add_argument(
        "--filtering_params_cfg",
        type=str,
        required=False,
        default="configs/filtering_params.yaml",
        help="Path to the filtering parameters configuration file",
    )

    return parser.parse_args()


def initialize_dependencies(args):
    """Initializes the dependencies of the node and returns them

    Args:
        args (argparse.Namespace): Arguments parsed from the command line

    Returns:
        Dict: Dictionary containing the dependencies of the node
    """
    subscriber = Subscriber(args.subscriber_cfg)
    publisher = Publisher(args.publisher_cfg)
    filtering_params = load_params(args.filtering_params_cfg)
    return {
        "subscriber": subscriber,
        "publisher": publisher,
        "filtering_params": filtering_params,
    }


def publish_debug_info(
    msg: PointCloud2,
    msg_fields: List[PointField],
    pcd_numpy: np.ndarray,
    debug_info_published: bool,
) -> bool:
    """Publishes the debug information of the point cloud

    Args:
        msg (PointCloud2): Point cloud message
        msg_fields (List[PointField]): List of point fields in the point cloud message
        pcd_numpy (np.ndarray): Point cloud data in numpy array
        debug_info_published (bool): Flag to check if the debug info has been published

    Returns:
        bool: True if the debug info has been published
    """
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

        logger.debug(f"Point cloud data shape: {pcd_numpy.shape}")
        logger.debug(f"Point cloud data dtype: {pcd_numpy.dtype}")
    return True


def filter_point_cloud(dependencies: Dict, publish_stats: bool = False):
    """Subscribes to the point cloud topic, filters the ground and noise points then publishes the filtered point cloud

    Args:
        dependencies (Dict): Dictionary containing the dependencies of the node
        publish_stats (bool, optional): Whether to publish the statistics of the point cloud. Defaults to False.
    """
    logger.info("Starting point cloud filtering")
    debug_info_published = False
    num_points, intensity, _range, reflectivity = [], [], [], []
    filtering_params = dependencies["filtering_params"]

    try:
        while not rospy.is_shutdown():
            pcd = dependencies["subscriber"].get_message()
            if pcd:
                msg_fields = [field.name for field in pcd.fields]
                pcd_numpy = ros_numpy.point_cloud2.pointcloud2_to_array(pcd)

                debug_info_published = publish_debug_info(
                    pcd, msg_fields, pcd_numpy, debug_info_published
                )

                if publish_stats:
                    is_finite = (
                        (pcd_numpy["x"] != 0)
                        & (pcd_numpy["y"] != 0)
                        & (pcd_numpy["z"] != 0)
                    )
                    total_finite = is_finite.sum()
                    num_points.append(total_finite)
                    reflectivity.append(pcd_numpy["reflectivity"])
                    intensity.append(pcd_numpy["intensity"])
                    _range.append(pcd_numpy["range"])

                pre_processed_pcd = pre_processor(
                    pcd, filtering_params["pre_processing_params"]
                )
                pcd_noise, filtered_pcd = noise_filter(
                    pre_processed_pcd,
                    filtering_params.get("noise_filtering_params", {}),
                )
                ground_plane_pcd, filtered_pcd = ground_plane_filter(
                    filtered_pcd, filtering_params["ground_plane_filtering_params"]
                )

                dependencies["publisher"].publish(
                    filtered_pcd=filtered_pcd, ground_plane_pcd=ground_plane_pcd
                )

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        logger.info("Shutting down")
    finally:
        if publish_stats:
            logger.info(
                "Statistics of the data (only points with finite x-y-z coordinates are considered)"
            )
            logger.info("Average number of points: {}".format(np.mean(num_points)))
            logger.info("Minimum number of points: {}".format(np.min(num_points)))
            logger.info("Maximum number of points: {}".format(np.max(num_points)))

            logger.info("Average reflectivity: {}".format(np.mean(reflectivity)))
            logger.info("Minimum reflectivity: {}".format(np.min(reflectivity)))
            logger.info("Maximum reflectivity: {}".format(np.max(reflectivity)))

            logger.info("Average intensity: {}".format(np.mean(intensity)))
            logger.info("Minimum intensity: {}".format(np.min(intensity)))
            logger.info("Maximum intensity: {}".format(np.max(intensity)))

            logger.info("Average range: {}".format(np.mean(_range)))
            logger.info("Minimum range: {}".format(np.min(_range)))
            logger.info("Maximum range: {}".format(np.max(_range)))


if __name__ == "__main__":
    args = parse_args()
    rospy.init_node(args.node_name)

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

    filter_point_cloud(dependencies)
