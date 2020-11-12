import os
from glob import glob
from setuptools import setup

package_name = 'ros_opcua_impl_freeopcua'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ]
)
