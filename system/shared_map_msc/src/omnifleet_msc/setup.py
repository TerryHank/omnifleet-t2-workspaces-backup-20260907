from setuptools import setup
setup(name='omnifleet_msc',version='0.2.0',packages=['omnifleet_msc'],
 data_files=[('share/ament_index/resource_index/packages',['resource/omnifleet_msc']),
 ('share/omnifleet_msc',['package.xml'])],
 entry_points={'console_scripts':['msc_coordinator=omnifleet_msc.coordinator:main',
 'msc_agent=omnifleet_msc.agent:main','msc=omnifleet_msc.cli:main',
 'msc_shared_map=omnifleet_msc.shared_map:main']})
