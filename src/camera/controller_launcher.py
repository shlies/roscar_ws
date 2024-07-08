#!/usr/bin/env python3

import os

def main():
    # ROS 2工作空间安装设置脚本路径
    ros2_ws_path = '/path/to/your/ros2_ws/install/setup.bash'

    # 源化ROS 2工作空间设置脚本
    source_cmd = f'source {ros2_ws_path} && '

    # 要运行的每个节点的命令
    nodes = [
        'arm_node',
        'controller_node',
        'fake_talker_node',
    ]

    # 构建启动每个节点的命令
    for node in nodes:
        cmd = source_cmd + f'ros2 run controller {node}'
        os.system(cmd)

if __name__ == '__main__':
    main()
