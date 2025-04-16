import glob
import os
from setuptools import find_packages, setup

package_name = 'data_collection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/data_collection/launch', glob.glob(os.path.join('launch', '*launch.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='piratestt@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab_5 = data_collection.lab_5.lab_5:main',
            'lab_5_sim_test = data_collection.lab_5.lab_5_sim_test:main',
            'lab_6 = data_collection.lab_6.lab_6:main',
            'lab_6_path = data_collection.lab_6.lab_6_path:main',
            'final = data_collection.final.final:main'
        ],
    },
)
