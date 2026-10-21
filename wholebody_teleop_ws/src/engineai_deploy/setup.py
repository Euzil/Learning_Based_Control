import os
from setuptools import find_packages, setup

package_name = 'engineai_deploy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *[
            (os.path.join('share', package_name, dirpath), [os.path.join(dirpath, f) for f in files if f.endswith('launch.py')])
            for dirpath, _, files in os.walk('launch')
            if any(f.endswith('launch.py') for f in files)
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devel',
    maintainer_email='simon.armleder@tum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'load_policy_node = engineai_deploy.load_policy_node:main',
            'multi_policy_runner_node = engineai_deploy.multi_policy_runner_node:main',
            'test_motors_node = engineai_deploy.test_motors_node:main',
        ],
    },
)
