from types import SimpleNamespace
from omnifleet_msc.coordinator import CoordinatorNode


def test_relocalization_requires_reverification_but_not_a_new_map():
    node=SimpleNamespace(shared_map={'sha256':'same-grid'},map_source_process='host:1234')
    reference={'localization_epoch':'host:1234:2','alignment_verified':True}
    assert CoordinatorNode.map_is_current(node,reference)
    reference['alignment_verified']=False
    assert not CoordinatorNode.map_is_current(node,reference)


def test_restart_cannot_reuse_the_previous_process_map():
    node=SimpleNamespace(shared_map={'sha256':'same-grid'},map_source_process='host:1234')
    reference={'localization_epoch':'host:5678:1','alignment_verified':True}
    assert not CoordinatorNode.map_is_current(node,reference)
    node.shared_map=None
    reference['localization_epoch']='host:1234:1'
    assert not CoordinatorNode.map_is_current(node,reference)
