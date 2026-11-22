from setuptools import find_packages, setup

setup(
    name="go1_mock",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/go1_mock"]),
        ("share/go1_mock", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go1 Interview Project",
    maintainer_email="go1-interview@example.invalid",
    description="Kinematic Go1 simulator and ROS 2 node",
    license="Apache-2.0",
    entry_points={"console_scripts": ["go1_mock_node = go1_mock.node:main"]},
)
