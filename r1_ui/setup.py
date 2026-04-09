from setuptools import find_packages, setup


package_name = "r1_ui"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    scripts=["scripts/r1_aruco_display_node"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "PyQt6",
        "opencv-contrib-python",
    ],
    zip_safe=True,
    maintainer="user",
    maintainer_email="yudai.yy0804@gmail.com",
    description="UI nodes for gakurobo2026_r1",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={},
)
