from setuptools import setup
import os
from glob import glob

package_name = 'pantograph_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eleftherios Trampas',
    maintainer_email='trampasleuteris@gmail.com',
    description='Pantograph 5-bar mechanism Webots simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pantograph_force = pantograph_pkg.ForceControl:main',
            'plot_results = pantograph_pkg.plot_force_results:main'
        ],
    },
)
