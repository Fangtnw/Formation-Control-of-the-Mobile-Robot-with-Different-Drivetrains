from setuptools import find_packages, setup

package_name = 'coop_controller'

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
    maintainer='fang',
    maintainer_email='thanawat.smkn@egmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "diffdrive_odom = coop_controller.diffdrive_odom:main",
            "mec_odom = coop_controller.mec_odom:main",
            "formation_controller = coop_controller.formation_controller:main",
            "odom_test = coop_controller.odom_test:main",
            "ack_odom = coop_controller.ack_odom:main",
            "sim_commander = coop_controller.sim_commander:main",
            "dynamic_footprint = coop_controller.dynamic_footprint:main",
            "coop_planner = coop_controller.coop_planner:main",
            "pure_pursuit_controller = coop_controller.pure_pursuit_controller:main",
            "pure_pursuit_controller_revised = coop_controller.pure_pursuit_controller_revised:main",
            "trajectory_plotter = coop_controller.trajectory_plotter:main",
            "hybrid_astar = coop_controller.hybrid_astar:main",
        ],
    },
)
