from setuptools import setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'rclpy',
        'std_msgs',
    ],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Maintainer Name',
    maintainer_email='maintainer.email@example.com',
    keywords=['ROS', 'ROS2', 'robotics'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Topic :: Software Development',
    ],
    description='ROS 2 package for controlling a robot arm and simulating coordinates',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_node = controller.arm:main',
            'controller_node = controller.controller:main',
            'fake_talker_node = controller.fake_talker:main',
        ],
    },
)
