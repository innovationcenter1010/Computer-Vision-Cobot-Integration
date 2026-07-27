from setuptools import setup

package_name = "cobot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/robot.launch.py"]),
        (
            f"share/{package_name}/config",
            ["config/cobot_params.yaml", "config/trajectories.json"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="james-t-smith",
    maintainer_email="innovationcenter1010@gmail.com",
    description="Launch files and configuration for the bin-picking stack.",
    license="MIT",
)
