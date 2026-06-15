// Copyright (c) 2024, RoboVerse community
// SPDX-License-Identifier: BSD-3-Clause

#include "lidar_processor_cpp/laser_scan_merger_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lidar_processor_cpp
{

LaserScanMergerNode::LaserScanMergerNode()
: Node("laser_scan_merger")
{
  declareParameters();
  setupPublishers();
  setupSubscriptions();

  RCLCPP_INFO(this->get_logger(),
    "Laser scan merger: %zu sources -> %s",
    scan_topics_.size(), output_topic_.c_str());
}

void LaserScanMergerNode::declareParameters()
{
  this->declare_parameter("scan_topics", std::vector<std::string>{
    "/scan/lidar_low",
    "/scan/lidar_high",
    "/scan/proximity"
  });
  this->declare_parameter("output_topic", "/scan");
  this->declare_parameter("max_age_sec", 0.5);

  scan_topics_ = this->get_parameter("scan_topics").as_string_array();
  output_topic_ = this->get_parameter("output_topic").as_string();
  max_age_sec_ = this->get_parameter("max_age_sec").as_double();
}

void LaserScanMergerNode::setupPublishers()
{
  auto qos = rclcpp::QoS(5)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);

  merged_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);
}

void LaserScanMergerNode::setupSubscriptions()
{
  auto qos = rclcpp::QoS(5)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);

  for (const auto& topic : scan_topics_) {
    auto sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      topic,
      qos,
      [this, topic](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scanCallback(msg, topic);
      }
    );
    subscriptions_.push_back(sub);
  }
}

bool LaserScanMergerNode::isValidRange(float range, float range_min, float range_max) const
{
  if (!std::isfinite(range)) {
    return false;
  }
  if (range < range_min || range > range_max) {
    return false;
  }
  return true;
}

void LaserScanMergerNode::mergeInto(
  sensor_msgs::msg::LaserScan& output,
  const sensor_msgs::msg::LaserScan& input) const
{
  if (output.ranges.empty() || input.ranges.empty()) {
    return;
  }

  const double output_inc = output.angle_increment;
  const double input_inc = input.angle_increment;
  if (output_inc <= 0.0 || input_inc <= 0.0) {
    return;
  }

  for (size_t out_idx = 0; out_idx < output.ranges.size(); ++out_idx) {
    const double angle = output.angle_min + static_cast<double>(out_idx) * output_inc;
    const int in_idx = static_cast<int>(std::lround((angle - input.angle_min) / input_inc));
    if (in_idx < 0 || static_cast<size_t>(in_idx) >= input.ranges.size()) {
      continue;
    }

    const float in_range = input.ranges[static_cast<size_t>(in_idx)];
    if (!isValidRange(in_range, input.range_min, input.range_max)) {
      continue;
    }

    float& out_range = output.ranges[out_idx];
    if (!isValidRange(out_range, output.range_min, output.range_max) || in_range < out_range) {
      out_range = in_range;
      if (out_idx < output.intensities.size() &&
          static_cast<size_t>(in_idx) < input.intensities.size()) {
        output.intensities[out_idx] = input.intensities[static_cast<size_t>(in_idx)];
      }
    }
  }
}

void LaserScanMergerNode::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg,
  const std::string& topic)
{
  latest_scans_[topic] = *msg;
  last_stamp_[topic] = rclcpp::Time(msg->header.stamp);
  publishMergedScan();
}

void LaserScanMergerNode::publishMergedScan()
{
  const auto now = this->get_clock()->now();
  sensor_msgs::msg::LaserScan merged;
  bool has_reference = false;

  for (const auto& topic : scan_topics_) {
    const auto scan_it = latest_scans_.find(topic);
    const auto stamp_it = last_stamp_.find(topic);
    if (scan_it == latest_scans_.end() || stamp_it == last_stamp_.end()) {
      continue;
    }

    if ((now - stamp_it->second).seconds() > max_age_sec_) {
      continue;
    }

    const auto& scan = scan_it->second;
    if (!has_reference) {
      merged = scan;
      merged.ranges.assign(merged.ranges.size(), std::numeric_limits<float>::infinity());
      if (!merged.intensities.empty()) {
        merged.intensities.assign(merged.intensities.size(), 0.0f);
      }
      has_reference = true;
    }

    mergeInto(merged, scan);
  }

  if (!has_reference) {
    return;
  }

  merged.header.stamp = now;
  merged_pub_->publish(merged);
}

}  // namespace lidar_processor_cpp

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<lidar_processor_cpp::LaserScanMergerNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Error running laser scan merger: " << e.what() << std::endl;
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
