# 功能介绍

基于深度学习的方法识别图像中引导线的中点并发布消息，使用模型为resnet18

# 使用方法

## 准备工作

具备TonyPi人形机器人，包含机器人本体、相机及RDK套件，并且能够正常运行。

## 编译与运行

**1.编译**

启动机器人后，通过终端SSH或者VNC连接机器人，点击本页面右上方的“一键部署”按钮，复制如下命令在RDK的系统上运行，完成相关Node的安装。

```bash
# 拉取赛道检测代码与图像矫正代码
mkdir -p ~/tonypi_ws/src && cd ~/tonypi_ws/src
git clone https://github.com/wunuo1/tonypi_line_detection.git -b feature-humble-x5
git clone https://github.com/wunuo1/tonypi_image_correction.git -b feature-humble-x5

#编译
cd ..
source /opt/tros/setup.bash
colcon build
```

**2.运行巡线感知功能**

```shell
source ~/tonypi_ws/install/setup.bash
cp -r ~/tonypi_ws/install/tonypi_line_detection/lib/tonypi_obj_detection/config/ .

# web端可视化引导线中点（启动功能后在浏览器打开 ip:8000）
export WEB_SHOW=TRUE

ros2 launch tonypi_line_detection line_center_detection.launch.py
```


# 原理简介

地平线RDK通过摄像头获取机器人前方环境数据，图像数据通过训练好的模型进行推理得到引导线的坐标值并发布。

# 接口说明

## 话题

### Pub话题

| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /line_center_detection        | ai_msgs::msg::PerceptionTargets               | 发布引导线中点的图像坐标                 |

### Sub话题
| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /hb_image                     | hbm_img_msgs/msg/HbmMsg1080P                                 | 图像矫正节点发布的图片消息（640x480）                   |

## 参数

| 参数名                | 类型        | 说明   |
| --------------------- | ----------- | -------------------------------------------------------------------------------------------------- |
| model_path       | string | 推理使用的模型文件，请根据实际模型路径配置，默认值为config/tonypi_line_center_detection_x5.bin |
| sub_img_topic    | string |  接收的图片话题名称，请根据实际接收到的话题名称配置，默认值为/hb_image |

# 注意
该功能包提供特定的实际场景中可使用的模型，若自行采集数据集进行训练，请注意替换。
