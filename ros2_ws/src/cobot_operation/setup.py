from setuptools import setup

package_name = "cobot_operation"

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
    description="Pick choreography node — chains cobot_control primitives into PickCan.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cobot_operation_node = cobot_operation.operation_node:main",
        ],
    },
)
