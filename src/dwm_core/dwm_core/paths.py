"""Resolve installed resources and keep writable state outside the workspace."""

import os
from pathlib import Path
import shutil
import tempfile


def package_share(name):
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(name))
    except (ImportError, LookupError):
        for parent in Path(__file__).resolve().parents:
            candidate = parent / "src" / name
            if (candidate / "package.xml").is_file():
                return candidate
        raise RuntimeError(f"Package {name} is unavailable; source install/setup.bash")


def runtime_root():
    default = (
        Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share"))
        / "dwm_robot"
    )
    root = Path(os.environ.get("DWM_DATA_DIR", default)).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    for name in ("logs", "maps", "routes"):
        (root / name).mkdir(exist_ok=True)
    for name in ("config.json", "navigation_geometry.json"):
        target = root / name
        if not target.exists():
            # Publish a complete default file once, even when both processes start
            # together. A hard link fails rather than replacing existing settings.
            descriptor, temporary_name = tempfile.mkstemp(prefix=".default-", dir=root)
            os.close(descriptor)
            temporary = Path(temporary_name)
            try:
                shutil.copyfile(
                    package_share("dwm_bringup") / "config" / name, temporary
                )
                try:
                    os.link(temporary, target)
                except FileExistsError:
                    pass
            finally:
                temporary.unlink(missing_ok=True)
    return root


def model_path(value):
    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        return candidate
    return package_share("dwm_perception") / "models" / candidate.name
