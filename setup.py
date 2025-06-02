from setuptools import find_packages, setup

package_name = "controlkitv3"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="advay",
    maintainer_email="advay.pakhale@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_display = controlkitv3.image_display:main",
            "service_calls = controlkitv3.service_calls:main",
            "teleop = controlkitv3.teleop:main",
        ],
    },
)
