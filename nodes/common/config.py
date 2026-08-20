"""
config.py — shared config loader for Python nodes.
Reads the same root config.json the C++ nodes use.
"""

import json
from pathlib import Path


def load_config(filename: str = "config.json") -> dict:
    """Walks upward from cwd looking for config.json so it doesn't
    matter exactly where the script is launched from."""
    candidate = Path(filename)
    for _ in range(5):
        if candidate.is_file():
            return json.loads(candidate.read_text())
        candidate = Path("..") / candidate
    raise FileNotFoundError(f"Could not find {filename} within 5 parent directories")


# Usage:
#   from config import load_config
#   cfg = load_config()
#   topic = cfg["ecal"]["topic_scene_comparisons"]
#   out_dir = cfg["logger"]["output_dir"]
