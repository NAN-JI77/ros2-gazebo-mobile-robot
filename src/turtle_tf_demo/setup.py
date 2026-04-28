from setuptools import setup
from glob import glob

package_name = 'turtle_tf_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='n',
    maintainer_email='n@example.com',
    description='turtle tf demo package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf_broadcaster = turtle_tf_demo.turtle_tf_broadcaster:main',
            'turtle_tf_listener = turtle_tf_demo.turtle_tf_listener:main',
            'turtle_tf_follower = turtle_tf_demo.turtle_tf_follower:main',
            'cmd_vel_relay = turtle_tf_demo.cmd_vel_relay:main',
            'auto_drive = turtle_tf_demo.auto_drive:main',
            'drive_distance = turtle_tf_demo.drive_distance:main',
            'drive_with_odom = turtle_tf_demo.drive_with_odom:main',
            'turn_with_odom = turtle_tf_demo.turn_with_odom:main',
            'square_with_odom = turtle_tf_demo.square_with_odom:main',
            'go_to_goal = turtle_tf_demo.go_to_goal:main',
            'follow_waypoints = turtle_tf_demo.follow_waypoints:main',
            'follow_waypoints_param = turtle_tf_demo.follow_waypoints_param:main',
            'go_to_goal_smooth = turtle_tf_demo.go_to_goal_smooth:main',
            'obstacle_avoid = turtle_tf_demo.obstacle_avoid:main',
            'waypoint_avoid = turtle_tf_demo.waypoint_avoid:main',
            'waypoint_avoid_sm = turtle_tf_demo.waypoint_avoid_sm:main',
        ],
    },
)
