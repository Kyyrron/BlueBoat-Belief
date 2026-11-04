from glob import glob

from setuptools import find_packages, setup

PACKAGE = "blueboat_belief"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="Killian Bertrand",
    maintainer_email="killian.bertrand2002@gmail.com",
    description="BlueBoat-Belief: probabilistic belief map for aspect-aware side-scan survey",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            f"belief_node = {PACKAGE}.ros.belief_node:main",
            f"blueboat-belief-replay = {PACKAGE}.cli.replay:main",
            f"blueboat-belief-make-area = {PACKAGE}.cli.make_area:main",
        ],
    },
)
