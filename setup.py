from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "flyto_robotics"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/config", glob("config/*")),
        (f"share/{PACKAGE_NAME}/contracts", glob("contracts/*.json")),
        (f"share/{PACKAGE_NAME}/examples/jobs", glob("examples/jobs/*.json")),
        (
            f"share/{PACKAGE_NAME}/examples/goal-frames",
            glob("examples/goal-frames/*.json"),
        ),
        (f"share/{PACKAGE_NAME}/examples/maps", glob("examples/maps/*.json")),
        (f"share/{PACKAGE_NAME}/examples/plans", glob("examples/plans/*.json")),
        (f"share/{PACKAGE_NAME}/examples/routes", glob("examples/routes/*.json")),
        (
            f"share/{PACKAGE_NAME}/examples/resource-plans",
            glob("examples/resource-plans/*.json"),
        ),
        (
            f"share/{PACKAGE_NAME}/examples/facility-resources",
            glob("examples/facility-resources/*.json"),
        ),
        (
            f"share/{PACKAGE_NAME}/examples/guarded-handoff",
            glob("examples/guarded-handoff/*.json"),
        ),
        (
            f"share/{PACKAGE_NAME}/scenarios/gazebo",
            glob("scenarios/gazebo/*.json"),
        ),
        (f"share/{PACKAGE_NAME}/worlds", glob("worlds/*.sdf")),
        (f"share/{PACKAGE_NAME}/models/flyto_rover", glob("models/flyto_rover/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Flyto2 Robotics",
    maintainer_email="support@flyto2.com",
    description="Flyto2 ROS 2, Gazebo, and physical robot boundary",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "flyto-robotics = flyto_robotics.cli:main",
            "gazebo_lab_driver = flyto_robotics.gazebo_lab_driver:main",
            "mission_controller = flyto_robotics.ros2_node:main",
            "robotics_planning_session = flyto_robotics.planning_session:main",
            "shortcut_controller = flyto_robotics.shortcut_ros2_node:main",
            "shortcut_gazebo_driver = flyto_robotics.shortcut_gazebo_driver:main",
            "showcase_gazebo_observer = flyto_robotics.showcase_gazebo_observer:main",
        ],
    },
)
