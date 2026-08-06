from setuptools import find_packages, setup


package_name = "obstacle_perception"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/launch.py"]),
        ("share/" + package_name + "/config", ["config/obstacle_perception.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UAEU Soccer Team",
    maintainer_email="team@uaeusoccer.invalid",
    description="Depth-based local obstacle perception for Booster Soccer.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "depth_obstacle_node = obstacle_perception.depth_obstacle_node:main",
        ],
    },
)
