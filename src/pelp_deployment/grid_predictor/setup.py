from setuptools import find_packages, setup
import os

package_name = "grid_predictor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/model", [os.path.join("model", "fpunet.pth")]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jingfan",
    maintainer_email="tangjingfan@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "grid_predictor_node = grid_predictor.grid_predictor_node:main"
        ],
    },
)
