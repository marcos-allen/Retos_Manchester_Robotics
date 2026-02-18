from setuptools import setup

package_name = 'reto_1_equipo_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/challenge_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanianwn',
    maintainer_email='tania.ab.mx@gmail.com',
    description='Signal generator and processor ROS2 challenge',
    license='TODO',
    entry_points={
        'console_scripts': [
            'signal_generator = reto_1_equipo_5.signal_generator:main',
            'signal_processor = reto_1_equipo_5.signal_processor:main',
        ],
    },
)

