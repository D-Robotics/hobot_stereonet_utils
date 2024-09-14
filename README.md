# hobot_stereonet_utils

## 描述

双目辅助功能包，可以对双目图像进行采集

## 编译

```shell
# X5交叉编译
bash ./robot_dev_config/build.sh -p X5 -s hobot_stereonet_utils
```

## 运行命令

```shell
# 在终端1运行以下命令，启动双目相机，输入http://ip:8000可以查看双目图像
ros2 launch hobot_stereonet_utils test_cam.launch.py

# 在终端2运行以下命令，可以对双目图像进行采集，按'Enter'键采集一张
ros2 run hobot_stereonet_utils hobot_stereonet_utils
```

## 运行结果

- 终端1结果

[](./doc/test_cam.png)

- 终端2结果

[](./doc/save_image.png)

## 使用案例

### 采集棋盘格图像

可以配合[双目标定GitHub仓库](https://github.com/D-Robotics/stereo_calib.git)使用，对双目相机进行标定，采集图像后，输入标定程序即可获得标定结果

[](./doc/display_and_save.png)

### 测试双目标定结果

将[双目标定GitHub仓库](https://github.com/D-Robotics/stereo_calib.git)运行结果生成的文件`stereo_8.yaml`文件复制到X5板端，然后运行

```shell
# 在终端2运行以下命令，启动双目功能包，输入http://ip:8000可以查看结果
ros2 launch hobot_stereonet_utils test_stereo.launch.py stereo_calib_path:=./stereo_8.yaml visual_alpha:=4
```

[](./doc/test_stereo.png)