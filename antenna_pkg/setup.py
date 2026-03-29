from setuptools import find_packages, setup

package_name = "antenna_pkg"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Riley McLain",
    maintainer_email="rjm0037@uah.edu",
    description="Tracking antenna ROS node",
    license="AGPL-3.0-only",
    entry_points={
        "console_scripts": [
            "antenna = src.antenna_node:main",
        ],
    },
)
