from pathlib import Path
from setuptools import find_packages, setup

package_name = "dwm_robot"
data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]
for folder in ("web",):
    for path in sorted(Path(folder).rglob("*")):
        if path.is_file():
            data_files.append(
                ("share/" + package_name + "/" + str(path.parent), [str(path)])
            )

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Rayan-01",
    maintainer_email="rayanalamsiam2000@gmail.com",
    url="https://github.com/Rayan-01/dwm-robot",
    description="Robot hardware runtime and browser dashboard.",
    license="LicenseRef-Proprietary",
    entry_points={"console_scripts": ["robot_server = dwm_robot.robot:main"]},
)
