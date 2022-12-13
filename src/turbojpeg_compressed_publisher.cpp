// Copyright 2022 Daisuke Nishimatsu
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

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <sstream>
#include <vector>

#include <cv_bridge/cv_bridge.h>  // NOLINT
#include <sensor_msgs/image_encodings.hpp>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter_client.hpp>

#include "turbojpeg_compressed_image_transport/turbojpeg_compressed_publisher.hpp"


namespace turbojpeg_compressed_image_transport
{

namespace enc = sensor_msgs::image_encodings;

void TurbojpegCompressedPublisher::advertiseImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos)
{
  node_ = node;
  using Base = image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage>;
  Base::advertiseImpl(node, base_topic, custom_qos);

  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  jpeg_quality_param_name_ = param_base_name + ".jpeg_quality";
  rcl_interfaces::msg::ParameterDescriptor jpeg_quality_description;
  jpeg_quality_description.name = "jpeg_quality";
  jpeg_quality_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  jpeg_quality_description.description = "Image quality for JPEG format";
  jpeg_quality_description.read_only = false;
  rcl_interfaces::msg::IntegerRange jpeg_range;
  jpeg_range.from_value = 1;
  jpeg_range.to_value = 100;
  jpeg_range.step = 1;
  jpeg_quality_description.integer_range.push_back(jpeg_range);
  try {
    config_.jpeg_quality = node->declare_parameter(
      jpeg_quality_param_name_, DEFAULT_JPEG_QUALITY, jpeg_quality_description);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", jpeg_quality_param_name_.c_str());
    config_.jpeg_quality = node->get_parameter(jpeg_quality_param_name_).get_value<int64_t>();
  }
}

void TurbojpegCompressedPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  // Compressed image message
  sensor_msgs::msg::CompressedImage compressed;
  compressed.header = message.header;
  compressed.format = message.encoding;

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(message.encoding);

  // Update ros message format header
  compressed.format += "; jpeg compressed ";

  // Check input format

  if ((bitDepth == 8) || (bitDepth == 16)) {
    // Target image format
    std::string targetFormat;
    int jpeg_subsample{};
    uint64_t jpeg_size{};
    unsigned char * jpeg_buffer = nullptr;
    if (enc::isColor(message.encoding)) {
      // convert color images to BGR8 format
      targetFormat = "bgr8";
      jpeg_subsample = TJPF_BGR;
      compressed.format += targetFormat;
    } else {
      // convert gray images to mono8 format
      targetFormat = "mono8";
      jpeg_subsample = TJPF_GRAY;
      compressed.format += targetFormat;
    }

    // OpenCV-ros bridge
    try {
      std::shared_ptr<TurbojpegCompressedPublisher> tracked_object;
      cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvShare(message, tracked_object, targetFormat);
      // Compress image
      tjCompress2(
        *tjhandle_,
        cv_ptr->image.datastart,
        message.width,
        0,
        message.height,
        jpeg_subsample,
        &jpeg_buffer,
        &jpeg_size,
        TJSAMP_420,
        config_.jpeg_quality,
        TJFLAG_FASTDCT
      );
      compressed.data.resize(jpeg_size);
      std::copy(jpeg_buffer, jpeg_buffer + jpeg_size, compressed.data.begin());
      tjFree(jpeg_buffer);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(logger_, "%s", e.what());
      return;
    } catch (cv::Exception & e) {
      RCLCPP_ERROR(logger_, "%s", e.what());
      return;
    }
    // Publish message
    publish_fn(compressed);
  } else {
    RCLCPP_ERROR(
      logger_,
      "Compressed Image Transport - "
      "JPEG compression requires 8/16-bit color format (input format is: %s)",
      message.encoding.c_str());
  }
}
}  // namespace turbojpeg_compressed_image_transport
