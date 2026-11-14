"""Project folders, found from this file's location (no hardcoded user paths)."""

from pathlib import Path

# script/mayakinect/paths.py -> project root is two folders up from script/
PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODELS_DIR = PROJECT_ROOT / "models"
THUMBS_DIR = MODELS_DIR / "thumbnails"
TAKES_DIR = PROJECT_ROOT / "takes"


def maya_path(path):
    """Return the path as a string with forward slashes, as Maya/MEL expects."""
    return str(path).replace("\\", "/")
