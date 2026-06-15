// Copyright (c) 2024, RoboVerse community
// SPDX-License-Identifier: BSD-3-Clause

#ifndef LIDAR_PROCESSOR_CPP__LASER_SCAN_MERGER_NODE_HPP_
#define LIDAR_PROCESSOR_CPP__LASER_SCAN_MERGER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace lidar_processor_cpp
{

class LaserScanMergerNode : public rclcpp::Node
{
public:
  LaserScanMergerNode();

private:
  void declareParameters();
  void setupSubscriptions();
  void setupPublishers();
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string& topic);
  void publishMergedScan();
  bool isValidRange(float range, float range_min, float range_max) const;
  void mergeInto(
    sensor_msgs::msg::LaserScan& output,
    const sensor_msgs::msg::LaserScan& input) const;

  std::vector<std::string> scan_topics_;
  std::string output_topic_;
  double max_age_sec_;

  std::unordered_map<std::string, sensor_msgs::msg::LaserScan> latest_scans_;
  std::unordered_map<std::string, rclcpp::Time> last_stamp_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_pub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subscriptions_;
};

}  // namespace lidar_processor_cpp

#endif  // LIDAR_PROCESSOR_CPP__LASER_SCAN_MERGER_NODE_HPP_
