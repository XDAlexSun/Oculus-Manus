from setuptools import find_packages, setup

package_name = 'robot_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/hand.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mezael Docoy',
    maintainer_email='kim1991mail@gmail.com',
    description='Robot Hand MuJoCo simulation bridge',
    license='BSD-3-Clause',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mujoco_hand_node = robot_sim.mujoco_hand_node:main',
        ],
    },
)
