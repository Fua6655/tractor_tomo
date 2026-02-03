from setuptools import setup
import os
from glob import glob

package_name = "tomo_factory"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@email.com",
    description="TOMO motion & control factories",
    license="MIT",
    entry_points={
        "console_scripts": [
            "control_factory = tomo_factory.control_factory:main",
            "motion_factory = tomo_factory.motion_factory:main",
        ],
    },
)
