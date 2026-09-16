"""
config.py
---------
Small utility to load the YAML experiment configuration from anywhere in
the project (scripts/, tests/, src/) without relying on the current
working directory.
"""
import os
import yaml

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.normpath(
    os.path.join(_THIS_DIR, "..", "config", "experiment_config.yaml")
)


def load_config(path: str = None) -> dict:
    path = path or DEFAULT_CONFIG_PATH
    with open(path, "r") as f:
        return yaml.safe_load(f)
