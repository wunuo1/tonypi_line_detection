# tonypi_line_detection
# Function Introduction

Using deep learning methods, this package identifies the midpoint of guide lines in images and publishes messages. The model used is resnet18.

# Usage

## Preparations

Have a TonyPi humanoid robot, including the robot body, camera, and RDK suite, and ensure it runs normally.

## Install the Package

**1. Install the package**

After starting the robot, connect to the robot through terminal SSH or VNC, click the "One-click Deployment" button at the top right of this page, copy the following command to run on the RDK system to complete the installation of the relevant Node.

```bash
sudo apt update
sudo apt install -y tros-tonypi-line-detection
```
**2. Run the Task Decomposition Function**

```shell
source /opt/tros/local_setup.bash

# Visualize the guide line midpoint on the web (after starting the function, open ip:8000 in the browser)
export WEB_SHOW=TRUE

ros2 launch tonypi_line_detection line_center_detection.launch.py
```

# Principle Overview
The RDK X3 obtains data from the environment in front of the robot through the camera. The image data is inferred using a trained model to get the coordinates of the guide line and publishes them.

# Interface Description

## Topics

### Published Topics

|Name  | Type                                  |  Description           |
|------| --------------------------------------| --------------------------------|
|/line_center_detection |ai_msgs::msg::PerceptionTargets | Publishes the image coordinates of the guide line midpoint|

### Subscribed Topics

|Name  | Type                                  |  Description           |
|------| --------------------------------------| --------------------------------|
|/hb_image |hbm_img_msgs/msg/HbmMsg1080P| Image message published by the image correction node (640x480)|


## Parameters
| Parameter Name             | Type       | Description  |
| --------------------- | ----------- | ----------------------------------------------------- |
| model_path	|string	|The model file used for inference. Configure according to the actual model path. Default value is /opt/nodehub_model/tonypi_detection/tonypi_line_center_detection.bin |
| sub_img_topic	|string	|The name of the subscribed image topic. Configure according to the actual received topic name. Default value is /hb_image |

# Note
This package provides a model that can be used in specific real-world scenarios. If you collect your own dataset for training, please replace the model accordingly.