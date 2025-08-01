from setuptools import find_packages, setup

package_name = 'agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olin',
    maintainer_email='olin.goog@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'keys = agent.policy_performer:just_keys',
        'state_former = agent.state_former:main',
        'rewarder = agent.command:main',
        'train = agent.policy_performer:teleop',
        ],
    },
)
