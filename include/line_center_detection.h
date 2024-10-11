// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LINE_CENTER_DETECTION_H_
#define LINE_CENTER_DETECTION_H_

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/dnn_node_data.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "sensor_msgs/msg/image.hpp"

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::ModelTaskType;

using hobot::dnn_node::DNNTensor;

class LineCoordinateResult{
 public:
  float x;
  float y;
  void Reset() {x = -1.0; y = -1.0;}
};

class LineCoordinateParser {
 public:
  LineCoordinateParser() {}
  ~LineCoordinateParser() {}
  int32_t Parse(
      std::shared_ptr<LineCoordinateResult>& output,
      std::shared_ptr<DNNTensor>& output_tensor);
};

class LineCenterDetectionNode : public DnnNode {
 public:
  LineCenterDetectionNode(const std::string& node_name,
                        const NodeOptions &options = NodeOptions());
  ~LineCenterDetectionNode() override;

 protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  int Predict(std::vector<std::shared_ptr<DNNInput>> &dnn_inputs,
              const std::shared_ptr<DnnNodeOutput> &output,
              const std::shared_ptr<std::vector<hbDNNRoi>> rois);
  void subscription_callback(
    const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg);
  bool GetParams();
  bool AssignParams(const std::vector<rclcpp::Parameter> & parameters);
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
    subscriber_hbmem_ = nullptr;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr publisher_ =
      nullptr;
  cv::Mat image_bgr_;
  std::string model_path_ = "config/tonypi_line_center_detection_x5.bin";
  std::string sub_img_topic_ = "/image_correct";
};

#endif  // LINE_CENTER_DETECTION_H_