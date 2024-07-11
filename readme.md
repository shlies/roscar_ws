# 机器人基础实践-视觉抓取 ros代码仓库

## Q&A

### 设备串口打不开

需要设置设备权限

在终端执行

    lsusb

找到 *ttyUSB* *设备

#### 单次设置（每次插拔/重启后需重新执行）

执行

    sudo chmod 777 /dev/ttyUSB* #刚才找到的设备名

设置完成

#### 一劳永逸（）

首先查看用户组

    ls -l /dev/ttyUSB*

终端输出：

    crw-rw---- 1 root dialout 188, 0  7月 11 20:27 /dev/ttyUSB0

查找用户名并将当前用户名添加到dialout用户组

    $ whoami
    shlies

    sudo usermod -aG dialout shlies

最后，重启电脑，设置完成
    
### USB端口号总是变

查看设备ID

    lsusb

例如

    Bus 001 Device 012: ID Bus 001 Device 012: ID 1a86:7523 QinHeng Electronics CH340 serial converter:7523 QinHeng Electronics CH340 serial converter

表示idVendor=1a86,idProduct=7523

修改端口规则

    sudo nano /etc/udev/rules.d/99-usb-serial.rules

添加：

    SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0660", GROUP="dialout, SYMLINK+="ttyUSB_car"

*拔出设备*

重启服务

    sudo udevadm control --reload-rules
    sudo udevadm trigger
