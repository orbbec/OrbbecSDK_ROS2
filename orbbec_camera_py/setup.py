import os
from glob import glob
from setuptools import find_packages, setup

package_name = "orbbec_camera_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Amritpal Saini",
    maintainer_email="asaini@locusobotics.com",
    description="Interfacing test nodes for orbbec camera",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "run_orbbec_trigger = orbbec_camera_py.run_orbbec_trigger:main",
        ],
    },
)
