from setuptools import setup

setup(name='omnifleet_fleet_control', version='0.1.0', packages=['omnifleet_fleet_control'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/omnifleet_fleet_control']),
                  ('share/omnifleet_fleet_control', ['package.xml', 'README.md'])],
      install_requires=['setuptools'], license='Apache-2.0',
      entry_points={'console_scripts':['msc_agent=omnifleet_fleet_control.agent:main']})
