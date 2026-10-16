"""Named occupancy maps in Nav2 YAML/PGM format and browser raster format."""

import base64
import math
from pathlib import Path
import re
import shutil
import tempfile

from PIL import Image
import yaml


class MapStore:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)

    def directory(self, name):
        if not isinstance(name, str) or not re.fullmatch(
            r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", name
        ):
            raise ValueError(
                "Map name must contain 1-64 letters, digits, underscores or hyphens"
            )
        target = self.root / name
        if target.is_symlink():
            raise ValueError("Symlink map directories are not supported")
        return target

    def list(self):
        return sorted(
            path.name
            for path in self.root.iterdir()
            if path.is_dir()
            and not path.is_symlink()
            and re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", path.name)
            and (path / "map.yaml").is_file()
        )

    @staticmethod
    def metadata(data):
        resolution = data["resolution"]
        origin = data["origin"]
        if (
            isinstance(resolution, bool)
            or not isinstance(resolution, (int, float))
            or not math.isfinite(resolution)
            or resolution <= 0
        ):
            raise ValueError("Map resolution must be finite and positive")
        if (
            not isinstance(origin, (list, tuple))
            or len(origin) != 3
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                for value in origin
            )
        ):
            raise ValueError("Map origin must contain finite x, y and yaw")
        free = float(data.get("free_thresh", 0.196))
        occupied = float(data.get("occupied_thresh", 0.65))
        negate = data.get("negate", 0)
        if not 0 <= free < occupied <= 1 or negate not in (0, 1):
            raise ValueError("Invalid map thresholds or negate value")
        return dict(
            resolution=float(resolution),
            origin=list(origin),
            negate=int(negate),
            free_thresh=free,
            occupied_thresh=occupied,
        )

    def read(self, name):
        directory = self.directory(name)
        data = yaml.safe_load((directory / "map.yaml").read_text())
        metadata = self.metadata(data)
        image_path = (directory / data["image"]).resolve()
        if not image_path.is_relative_to(directory.resolve()):
            raise ValueError("Map image must be inside its map directory")
        with Image.open(image_path) as image:
            if not 0 < image.width * image.height <= 4_000_000:
                raise ValueError("Map is too large")
            pixels = image.convert("L").tobytes()
            width, height = image.size
        return dict(
            name=name,
            width=width,
            height=height,
            metadata=metadata,
            pixels=base64.b64encode(pixels).decode("ascii"),
        )

    def save(self, name, grid):
        target = self.directory(name)
        if target.exists():
            raise ValueError("Map already exists; choose a new name")
        width, height = grid["width"], grid["height"]
        if (
            any(
                isinstance(v, bool) or not isinstance(v, int) or v <= 0
                for v in (width, height)
            )
            or width * height > 4_000_000
        ):
            raise ValueError("Invalid map dimensions")
        pixels = base64.b64decode(grid["pixels"], validate=True)
        if len(pixels) != width * height:
            raise ValueError("Map pixels do not match dimensions")
        metadata = self.metadata(grid["metadata"])
        temporary = Path(tempfile.mkdtemp(prefix="." + name + "-", dir=self.root))
        try:
            with (temporary / "map.pgm").open("wb") as stream:
                stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
                stream.write(pixels)
            (temporary / "map.yaml").write_text(
                yaml.safe_dump(
                    dict(image="map.pgm", mode="trinary", **metadata), sort_keys=False
                )
            )
            temporary.rename(target)
        except Exception:
            shutil.rmtree(temporary)
            raise
        return self.read(name)
