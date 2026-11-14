"""Find the character models that can be imported."""

import json
from dataclasses import dataclass, field
from pathlib import Path

from .. import paths

MODELS_JSON = paths.MODELS_DIR / "models.json"


@dataclass
class ModelInfo:
    name: str                       # file name without extension, used for node names
    obj: Path                       # path to the .obj file
    thumb: Path = None              # path to the .png thumbnail, None if missing
    display_name: str = ""          # label shown in the UI
    scale: float = 1.0
    offset: tuple = field(default=(0.0, 0.0, 0.0))

    def __post_init__(self):
        if not self.display_name:
            self.display_name = self.name


def _load_overrides():
    if not MODELS_JSON.exists():
        return {}
    try:
        with open(MODELS_JSON, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError) as e:
        print("mayakinect: could not read {}: {}".format(MODELS_JSON, e))
        return {}


def from_path(obj_path, overrides=None):
    """Build a ModelInfo for any .obj file (also outside models/)."""
    obj_path = Path(obj_path)
    name = obj_path.stem
    thumb = paths.THUMBS_DIR / (name + ".png")
    settings = (overrides or {}).get(name, {})
    return ModelInfo(
        name=name,
        obj=obj_path,
        thumb=thumb if thumb.exists() else None,
        display_name=settings.get("display_name", ""),
        scale=float(settings.get("scale", 1.0)),
        offset=tuple(settings.get("offset", (0.0, 0.0, 0.0))),
    )


def discover():
    """All models in models/, sorted by name."""
    overrides = _load_overrides()
    return [from_path(p, overrides) for p in sorted(paths.MODELS_DIR.glob("*.obj"))]
