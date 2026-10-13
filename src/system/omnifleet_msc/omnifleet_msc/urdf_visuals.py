"""Compatibility import; implementation belongs to omnifleet_fleet_visualization."""
import importlib as _importlib
import sys as _sys

_implementation = _importlib.import_module('omnifleet_fleet_visualization.urdf_visuals')
if __name__ == '__main__':
    _implementation.main()
else:
    _sys.modules[__name__] = _implementation
