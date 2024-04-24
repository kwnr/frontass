from setuptools import find_packages, setup
import os
import glob

package_name = 'frontass'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/ui.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kwnr',
    maintainer_email='kwnr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui = frontass.ui:main'
        ],
    },
)
