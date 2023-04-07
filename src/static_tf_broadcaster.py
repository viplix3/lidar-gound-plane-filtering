import rospy
import tf2_ros
import geometry_msgs.msg
import argparse
import yaml


def load_params(param_file):
    with open(param_file, "r") as f:
        params = yaml.safe_load(f)
    return params


def parse_args():
    parser = argparse.ArgumentParser(description="Static TF broadcaster")
    parser.add_argument(
        "--config",
        type=str,
        required=False,
        default="config/static_tf_broadcaster.yaml",
        help="Path to the configuration file",
    )
    return parser.parse_args()


# Broadcast a static transform
def broadcast_static_tf(params):
    # Initialize a ROS node
    rospy.init_node("static_tf_broadcaster")

    # Create a static transform broadcaster
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create a TransformStamped message for the static transform
    static_transform = geometry_msgs.msg.TransformStamped()

    # Set the header and fields of the TransformStamped message
    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = params["parent_frame_id"]
    static_transform.child_frame_id = params["child_frame_id"]
    static_transform.transform.translation.x = params["translation"]["x"]
    static_transform.transform.translation.y = params["translation"]["y"]
    static_transform.transform.translation.z = params["translation"]["z"]
    static_transform.transform.rotation.x = params["rotation"]["x"]
    static_transform.transform.rotation.y = params["rotation"]["y"]
    static_transform.transform.rotation.z = params["rotation"]["z"]
    static_transform.transform.rotation.w = params["rotation"]["w"]

    # Broadcast the static transform
    broadcaster.sendTransform(static_transform)

    # Keep the node running and processing messages until shut down
    rospy.spin()


if __name__ == "__main__":
    args = parse_args()
    params = load_params(args.config)

    # Broadcast the static transform with the given parameters
    broadcast_static_tf(params)
