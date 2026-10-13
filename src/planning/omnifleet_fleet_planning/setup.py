from setuptools import setup

setup(name='omnifleet_fleet_planning', version='0.1.0', packages=['omnifleet_fleet_planning'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/omnifleet_fleet_planning']),
                  ('share/omnifleet_fleet_planning', ['package.xml', 'README.md']),
                  ('share/omnifleet_fleet_planning/launch', ['launch/nav2_direct.launch.py']),
                  ('share/omnifleet_fleet_planning/config', ['config/nav2_t2_rep105.yaml'])],
      install_requires=['setuptools'], license='Apache-2.0 AND MIT')
