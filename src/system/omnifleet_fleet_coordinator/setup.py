from setuptools import find_packages, setup

package_name = "omnifleet_fleet_coordinator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/fleet_coordinator.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "fleet_coordinator = omnifleet_fleet_coordinator.node:main",
        ],
    },
)
