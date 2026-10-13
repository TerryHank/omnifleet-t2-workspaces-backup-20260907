"""Compatibility import; implementation belongs to omnifleet_map_tf_generator."""
import importlib as _importlib
import sys as _sys

_implementation = _importlib.import_module('omnifleet_map_tf_generator.shared_map_node')
if __name__ == '__main__':
    _implementation.main()
else:
    _sys.modules[__name__] = _implementation
