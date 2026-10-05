"""Check the consolidation candidate without ROS or modifying the source tree."""
import ast
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[3]


def main():
    packages = {}
    for base, dirs, files in os.walk(ROOT, followlinks=False):
        directory = Path(base)
        if any(name in files for name in ("COLCON_IGNORE", "AMENT_IGNORE", "CATKIN_IGNORE")):
            dirs[:] = []
            continue
        if "package.xml" in files:
            name = ET.parse(directory / "package.xml").findtext("name")
            packages.setdefault(name, []).append(directory.relative_to(ROOT).as_posix())
            dirs[:] = []
    duplicates = {name: paths for name, paths in packages.items() if len(paths) > 1}
    assert not duplicates, duplicates
    manifest = Path(__file__).with_name("ACTIVE_MOLA_PACKAGES.tsv")
    active = [line.split("\t") for line in manifest.read_text(encoding="utf-8").splitlines()[1:]]
    assert len(active) == 37
    for name, version, source, destination in active:
        assert packages[name] == [source], (name, packages.get(name))
        assert ET.parse(ROOT / source / "package.xml").findtext("version") == version
        assert destination == "src/" + name
    legacy = ROOT / "localization/_legacy_mola_3_2_pending_validation"
    assert (legacy / "COLCON_IGNORE").is_file()
    assert len(list((legacy / "packages").glob("*/package.xml"))) == 24

    parsed_python = 0
    for package in [ROOT / row[2] for row in active] + [
        ROOT / "localization/omnifleet_localization",
        ROOT / "localization/omnifleet_t2_mola_experiments",
    ]:
        for path in package.rglob("*.launch.py"):
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            parsed_python += 1
    files = [
        "localization/mola_state_estimation_smoother/params/state-estimation-smoother.yaml",
        "localization/mola_state_estimation_smoother/mola-cli-launchs/state_estimator_ros2.yaml",
        "localization/mola_lidar_odometry/mola-cli-launchs/lidar_odometry_ros2.yaml",
        "localization/omnifleet_t2_mola_experiments/config/smoother_lio_only.yaml",
        "localization/omnifleet_localization/config/modes.yaml",
    ]
    for name in files:
        yaml.safe_load((ROOT / name).read_text(encoding="utf-8"))
    for filename in ("SOURCE_PATHS.tsv", "PATH_RELOCATION.tsv", "system/MULTI_ROBOT_ZENOH_PATHS.tsv"):
        rows = (ROOT / filename).read_text(encoding="utf-8").splitlines()[1:]
        missing = [row.split("\t")[1] for row in rows if not os.path.lexists(ROOT / row.split("\t")[1])]
        assert not missing, (filename, missing[:10])

    print(json.dumps({"active_ros_packages": len(packages), "active_mola_packages": len(active),
                      "duplicate_package_names": len(duplicates), "legacy_packages_preserved": 24,
                      "launch_python_parsed": parsed_python, "yaml_parsed": len(files),
                      "cpp_build": "PENDING", "robot_acceptance": "PENDING",
                      "legacy_deletion_allowed": False}, indent=2))


if __name__ == "__main__":
    main()
