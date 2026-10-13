"""Compatibility import; implementation belongs to omnifleet_fleet_planning."""
import importlib as _importlib
import sys as _sys

_implementation = _importlib.import_module('omnifleet_fleet_planning.task_board')
if __name__ == '__main__':
    _implementation.main()
else:
    _sys.modules[__name__] = _implementation
