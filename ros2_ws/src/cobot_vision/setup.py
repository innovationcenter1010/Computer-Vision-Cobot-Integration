from setuptools import setup

package_name = "cobot_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="james-t-smith",
    maintainer_email="innovationcenter1010@gmail.com",
    description="YOLO-OBB detection (yolo_node) and pixel-to-robot-space localization (localization_node).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_node = cobot_vision.yolo_node:main",
            "localization_node = cobot_vision.localization_node:main",
        ],
    },
)
