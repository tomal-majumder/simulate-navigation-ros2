from setuptools import setup

package_name = 'fsm_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tomal',
    maintainer_email='tomalmajumder094@email.com',
    description='FSM-based obstacle avoidance using LIDAR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fsm_navigation_node = fsm_navigation.fsm_navigation_node:main',
        ],
    },
)

