from setuptools import setup

package_name = "cobot_health"

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
    description="Heartbeat helper + health_monitor aggregation node.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "health_monitor = cobot_health.health_monitor_node:main",
        ],
    },
)
