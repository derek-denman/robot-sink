from glob import glob
from setuptools import setup

package_name = "base_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@localhost",
    description="Jetson base bringup bridge for BBB odometry + TF",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bbb_odom_tf_bridge = base_bringup.bbb_odom_tf_bridge:main",
        ],
    },
)
