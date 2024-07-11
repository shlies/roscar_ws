from setuptools import find_packages, setup

package_name = 'arm_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='principle',
    maintainer_email='2759730749@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'uart_arm_comm = arm_comm.uart_arm_comm:main',
        'uart_arm_comm_test = arm_comm.uart_arm_comm_test:main',
        ],
    },
)
