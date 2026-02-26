from glob import glob
from setuptools import setup

package_name = "toy_shoe_perception"

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
    description="Toy-vs-shoe perception tools and ROS 2 runtime",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trt_detector_node = toy_shoe_perception.trt_detector_node:main",
            "safety_gate_node = toy_shoe_perception.safety_gate_node:main",
        ],
    },
)
