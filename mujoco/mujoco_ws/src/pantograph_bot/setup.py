from setuptools import find_packages, setup

package_name = 'pantograph_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', [
            'models/pantograph_torque.xml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eleftherios Trampas',
    maintainer_email='trampasleuteris@gmail.com',
    description='MuJoCo Pantograph Bot ROS 2 Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_torque = pantograph_bot.mujoco_torque_node:main',
            'controller_force = pantograph_bot.ForceControl:main',
            'plot_force_results = pantograph_bot.plot_force_results:main',
        ],
    },
)
