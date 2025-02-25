from setuptools import setup

import glob

package_name = "flir_camera_driver"

setup(
    name=package_name,
    version="0.2.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob.glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thomas Mohren",
    maintainer_email="tlmohren@gmail.com",
    description=(
        "Publishes an image stream from a FLIR camera."
    ),
    license="LGPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "publish_camera = flir_camera_driver.publish_camera:main",
            'ping_cameras = flir_camera_driver.display_each_camera:main',
        ],
    },
)
