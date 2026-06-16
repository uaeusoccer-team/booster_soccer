from setuptools import find_packages, setup

package_name = "perception_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Booster Robotics",
    maintainer_email="chengcheng@booster.tech",
    description="Small perception topic bridges for RoboCup demo integration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ball_to_detection_bridge = perception_bridge.ball_to_detection_bridge:main",
        ],
    },
)
