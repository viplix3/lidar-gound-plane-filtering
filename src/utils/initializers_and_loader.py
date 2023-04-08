import yaml
from typing import Dict


def load_params(param_file: str) -> Dict:
    """Load parameters from a yaml file

    Args:
        param_file (str): Path to the yaml file
    Returns:
        params (Dict): Dictionary of parameters
    """
    with open(param_file, "r") as f:
        params = yaml.safe_load(f)
    return params
