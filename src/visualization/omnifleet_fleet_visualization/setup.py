from setuptools import setup

setup(name='omnifleet_fleet_visualization', version='0.1.0', packages=['omnifleet_fleet_visualization'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/omnifleet_fleet_visualization']),
                  ('share/omnifleet_fleet_visualization', ['package.xml', 'README.md'])],
      install_requires=['setuptools'], license='Apache-2.0',
      entry_points={'console_scripts':['foxglove_layer_offset.py=omnifleet_fleet_visualization.foxglove_layer_offset:main']})
