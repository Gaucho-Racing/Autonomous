from setuptools import setup

package_name = "cone_nav"

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
    maintainer="Adi",
    maintainer_email="adi@example.com",
    description="Python path planner node for cone_nav.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "path_planner_node = cone_nav.path_planner_node:main",
        ],
    },
)
