import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
LAYOUT = ROOT / "visualization/omnifleet_t2_ws/foxglove/fleet_panel/layout-updated.json"


def walk(value):
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from walk(item)
    elif isinstance(value, list):
        for item in value:
            yield from walk(item)


def test_layout_has_one_shared_3d_scene():
    data = json.loads(LAYOUT.read_text(encoding="utf-8"))
    config = data["configById"]
    scene_ids = [key for key in config if key.startswith("3D!")]
    assert scene_ids == ["3D!unifiedMap"]
    references = list(walk(data["layout"]))
    assert references.count("3D!unifiedMap") == 1
    assert "Tab!fleetScenes" not in references


def test_unified_scene_uses_fleet_map_and_single_ingress_topics():
    scene = json.loads(LAYOUT.read_text(encoding="utf-8"))["configById"]["3D!unifiedMap"]
    assert scene["followTf"] == "fleet_map"
    assert scene["topics"]["/fleet/map"]["visible"] is True
    assert scene["topics"]["/fleet/ui/markers"]["visible"] is True
    assert scene["topics"]["/fleet/robots"]["visible"] is False
    assert scene["topics"]["/robot_113/map"]["visible"] is False
    assert scene["publish"]["poseTopic"] == "/fleet/ui/add_pose"
    assert scene["publish"]["poseEstimateTopic"] == "/fleet/ui/set_initialpose"
    assert all(layer["layerId"] != "foxglove.Urdf" for layer in scene["layers"].values())
