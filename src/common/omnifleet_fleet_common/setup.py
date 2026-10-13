from setuptools import setup

setup(name='omnifleet_fleet_common', version='0.1.0', packages=['omnifleet_fleet_common'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/omnifleet_fleet_common']),
                  ('share/omnifleet_fleet_common', ['package.xml', 'README.md'])],
      install_requires=['setuptools'], license='Apache-2.0')
