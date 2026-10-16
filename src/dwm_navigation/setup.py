from setuptools import find_packages, setup

package_name = "dwm_navigation"
data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]

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
    description="Lidar mapping, odometry and waypoint navigation bridge.",
    license="LicenseRef-Proprietary",
    entry_points={
        "console_scripts": ["navigation_bridge = dwm_navigation.navigation_view:main"]
    },
)
