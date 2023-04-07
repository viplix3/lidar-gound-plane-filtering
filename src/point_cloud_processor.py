import logging
import argparse

from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Static TF broadcaster")
    parser.add_argument(
        "--config",
        type=str,
        required=False,
        default="../config/static_tf_broadcaster.yaml",
        help="Path to transform configuration file",
    )
    return parser.parse_args()


def initialize_dependencies(args):
    raise NotImplementedError


def filter_point_cloud(args):
    raise NotImplementedError


if __name__ == "__main__":
    args = parse_args()

    logfile_dir = Path(__file__).parent.parent / "logs"
    logfile_dir.mkdir(parents=True, exist_ok=True)

    logging.basicConfig(
        filename=logfile_dir / "app.log",
        filemode="w",
        level=args.log_level.upper(),
        format="%(asctime)s [%(levelname)8s] [%(filename)s:%(lineno)d]: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    logger = logging.getLogger(__name__)

    initialize_dependencies(args)
    filter_point_cloud(args)
