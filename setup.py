from setuptools import find_packages, setup

package_name = 'project_pegasus_isaac_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project maintainer',
    maintainer_email='maintainer@example.com',
    description='Aerostack2 Pegasus project and ROS camera-to-UDP video relay.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'stream_video_udp = project_pegasus_isaac_sim.stream_video_udp:main',
            # Preserve the command used by existing tmuxinator configurations.
            'stream_video_udp.py = project_pegasus_isaac_sim.stream_video_udp:main',
        ],
    },
)
