# 功能介绍

hobot_arm package 是基于 mono2d_trash_detection package 开发的2D垃圾目标检测+机械臂抓取的应用示例。在地平线的旭日X3派上利用BPU进行模型推理获得感知结果, 利用幻尔机械臂作为下位机, 进行垃圾抓取的示例。


# 编译

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.04
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### Ubuntu板端编译X3

1. 编译环境确认 
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
2. 编译

 编译命令：`colcon build --packages-select hobot_arm`

### Docker交叉编译X3

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

   ```
   export TARGET_ARCH=aarch64
   export TARGET_TRIPLE=aarch64-linux-gnu
   export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

   colcon build --packages-select hobot_arm \
      --merge-install \
      --cmake-force-configure \
      --cmake-args \
      --no-warn-unused-cli \
      -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
   ```

### X86 Ubuntu系统上编译X86版本

1. 编译环境确认

   - x86 ubuntu版本: ubuntu20.04

2. 编译

   - 编译命令：

   ```
   colcon build --packages-select hobot_arm  \
      --merge-install \
      --cmake-args \
      -DTHIRD_PARTY=`pwd`/../sysroot_docker \
   ```

## 注意事项

# 使用介绍

## 依赖

- mono2d_trash_detection package: 发布垃圾检测感知msg

## 参数

无

## 运行

编译成功后，将生成的install路径拷贝到地平线旭日X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：


### **X3 Ubuntu 启动**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono2d_trash_detection/config/ .

# 启动dnn_node_example package
# mipi摄像头输入检测，渲染结果在Web页面可视化
export CAM_TYPE=mipi
ros2 launch dnn_node_example dnn_node_example.launch.py config_file:=config/ppyoloworkconfig.json msg_pub_topic_name:=ai_msg_mono2d_trash_detection image_width:=1920 image_height:=1080

# 启动dnn_node_example package
# usb摄像头输入检测，渲染结果在Web页面可视化
export CAM_TYPE=usb
ros2 launch dnn_node_example dnn_node_example.launch.py config_file:=config/ppyoloworkconfig.json msg_pub_topic_name:=ai_msg_mono2d_trash_detection image_width:=1920 image_height:=1080

ros2 run hobot_arm hobot_arm


### **X3 Linux**
如果需要在PC端浏览器上渲染显示sensor发布的图片和对应的AI结果，确认旭日X3派已经启动用于web展示的webserver服务（设备启动后只需要启动一次服务，只有设备重启的情况下需要重新启动服务）。旭日X3派执行ps -aux命令查看是否有nginx进程，如果有表示已经启动此服务，如果无，启动服务，运行方法为：
```shell
cd /opt/tros/lib/websocket/webservice/
chmod +x ./sbin/nginx && ./sbin/nginx -p .
cd -
```

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r ./install/lib/mono2d_trash_detection/config/ .

# 启动图片发布pkg
./install/lib/mipi_cam/mipi_cam --ros-args -p out_format:=nv12 -p image_width:=416 -p image_height:=416 -p io_method:=shared_mem -p video_device:=GC4663 --log-level error &

# 启动JPEG图片编码&发布pkg
./install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &

# 启动web展示pkg
./install/lib/websocket/websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/ai_msg_mono2d_trash_detection --log-level error &

# 启动dnn_node_example node
./install/lib/dnn_node_example/example  --ros-args -p feed_type:=1 -p is_shared_mem_sub:=1 -p dump_render_img:=0 -p msg_pub_topic_name:=/ai_msg_mono2d_trash_detection --log-level warn &

# 启动hobot node
./install/lib/hobot_arm/hobot_arm --log-level warn
```

### **X86 Ubuntu**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r ./install/lib/mono2d_trash_detection/config/ .

# 启动dnn_node_example node
ros2 run dnn_node_example example --ros-args -p feed_type:=0 -p image:=config/trashDet0028.jpg -p image_type:=0 -p dump_render_img:=1 -p config_file:=config/ppyoloworkconfig.json

```

# 结果分析
## X3结果展示

输出log日志：
```
[WARN] [1700107618.608870258] [arm_node]: Parameter:
 targetX: 0.15
 targetY: 0
 targetZ: -0.08
[WARN] [1700107618.609489260] [arm_node]: ArmNode Init Succeed!
[WARN] [1700107621.610287833] [arm_node]: Operate Succeed!
^C[INFO] [1700107623.664628627] [rclcpp]: signal_handler(signal_value=2)
root@ubuntu:~# ros2 run hobot_arm hobot_arm

        This is hobot arm package.

============================================
        the robotic arm device
============================================

[WARN] [1700107629.060302603] [arm_node]: Parameter:
 targetX: 0.15
 targetY: 0
 targetZ: -0.08
[WARN] [1700107629.060854068] [arm_node]: ArmNode Init Succeed!
[WARN] [1700107632.061743344] [arm_node]: Operate Succeed!

```


## 结果说明
示例中展示了垃圾检测的实时推理效果。本地效果展示中展示了将回灌图片渲染保存在本地。

设置"dump_render_img"为1时，渲染图片实时保持在当前路径下。
