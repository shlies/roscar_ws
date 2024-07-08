from setuptools import setup

package_name = 'stm32_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for UART communication with STM32',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uart_comm = stm32_comm.uart_comm:main',
            'uart_comm_test = stm32_comm.uart_comm_test:main',
            'uart_comm_test_pub = stm32_comm.uart_comm_test_pub:main',
        ],
    },
)

