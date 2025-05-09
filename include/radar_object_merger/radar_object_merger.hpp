#ifndef RADAR_OBJECT_MERGER__RADAR_OBJECT_MERGER_HPP_
#define RADAR_OBJECT_MERGER__RADAR_OBJECT_MERGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <chrono>

namespace radar_object_merger
{

using autoware_perception_msgs::msg::DetectedObjects;
using std_msgs::msg::Float32;

class RadarObjectMerger : public rclcpp::Node
{
public:
  explicit RadarObjectMerger(const rclcpp::NodeOptions & options);

private:
  // Callbacks
  void on_velocity(const Float32::ConstSharedPtr msg);
  void on_radar_input(const DetectedObjects::ConstSharedPtr msg, const std::string & topic_name);

  // Publisher
  rclcpp::Publisher<DetectedObjects>::SharedPtr objects_pub_;

  // Subscribers
  rclcpp::Subscription<Float32>::SharedPtr velocity_sub_;
  std::vector<rclcpp::Subscription<DetectedObjects>::SharedPtr> radar_subs_;

  // Parameters
  double velocity_threshold_kmph_;
  double merge_frequency_hz_;
  double message_timeout_sec_;
  std::string merge_frame_;
  std::vector<std::string> radar_topics_;
  std::string front_center_radar_topic_;

  // Internal state
  double current_velocity_kmph_ = 0.0;
  std::map<std::string, DetectedObjects::ConstSharedPtr> radar_objects_;
  std::recursive_mutex data_mutex_;
  rclcpp::Time last_merge_time_;
  rclcpp::Duration message_timeout_duration_;
  bool merge_all_radars_ = true;

  // Timer for merging and publishing
  rclcpp::TimerBase::SharedPtr merge_timer_;
  void merge_and_publish();
};

}  // namespace radar_object_merger

#endif  // RADAR_OBJECT_MERGER__RADAR_OBJECT_MERGER_HPP_
