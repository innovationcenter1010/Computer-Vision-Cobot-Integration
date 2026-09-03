from setuptools import setup

package_name = "cobot_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # UI assets served by FastAPI; located at runtime via
        # ament_index get_package_share_directory("cobot_web").
        (f"share/{package_name}/templates", ["templates/index.html", "templates/dev.html"]),
        (
            f"share/{package_name}/static",
            [
                "static/app.js",
                "static/commands.js",
                "static/dev.js",
                "static/replay.js",
                "static/style.css",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="james-t-smith",
    maintainer_email="innovationcenter1010@gmail.com",
    description="FastAPI web UI bridged into the ROS graph.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "web_node = cobot_web.web_node:main",
        ],
    },
)
