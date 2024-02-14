from setuptools import setup

package_name = 'coop_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abd',
    maintainer_email='abd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_diff= my_robot_bringup.diffdrive_spawn.launch.py:main',
            'spawn_mec= my_robot_bringup.mecanum_spawn.launch.py:main',
            'diff_launch= diff.launch.py:main',
            'pi_diff_launch= pi_diff.launch.py:main',
            'mec_launch= mec.launch.py:main',
            'pi_mec_launch= pi_mec.launch.py:main'
        ],
    },
)
