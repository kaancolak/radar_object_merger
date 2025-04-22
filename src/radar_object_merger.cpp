#include "radar_object_merger/radar_object_merger.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>

namespace radar_object_merger
{

using std::placeholders::_1;

RadarObjectMerger::RadarObjectMerger(const rclcpp::NodeOptions & options)
: Node("radar_object_merger", options),
  last_merge_time_(0, 0, this->get_clock()->get_clock_type()),
  message_timeout_duration_(0, 0)
{
  velocity_threshold_kmph_ = this->declare_parameter<double>("velocity_threshold_kmph", 5.0);
  radar_topics_ = this->declare_parameter<std::vector<std::string>>("radar_topics", std::vector<std::string>());
  front_center_radar_topic_ = this->declare_parameter<std::string>("front_center_radar_topic", "");
  merge_frequency_hz_ = this->declare_parameter<double>("merge_frequency_hz", 10.0);
  message_timeout_sec_ = this->declare_parameter<double>("message_timeout_sec", 0.02);
  message_timeout_duration_ = rclcpp::Duration::from_seconds(message_timeout_sec_);
  merge_frame_ = this->declare_parameter<std::string>("merge_frame", "base_link");

  if (radar_topics_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'radar_topics' is empty. No radar data will be processed.");
    return;
  }
  if (front_center_radar_topic_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Parameter 'front_center_radar_topic' is empty. High speed mode (event-based merging) will not function.");
  } else {
    bool found = false;
    for(const auto& topic : radar_topics_) {
      if (topic == front_center_radar_topic_) {
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_WARN(this->get_logger(), "Front center topic '%s' not found in 'radar_topics'.", front_center_radar_topic_.c_str());
    }
  }

  objects_pub_ = this->create_publisher<DetectedObjects>("~/output/objects", rclcpp::QoS(10));

  velocity_sub_ = this->create_subscription<Float32>(
    "~/input/velocity", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&RadarObjectMerger::on_velocity, this, _1));

  for (const auto & topic_name : radar_topics_) {
    auto sub = this->create_subscription<DetectedObjects>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this, topic_name](const DetectedObjects::ConstSharedPtr msg) {
        this->on_radar_input(msg, topic_name);
      });
    radar_subs_.push_back(sub);
    RCLCPP_INFO(this->get_logger(), "Subscribing to radar topic: %s", topic_name.c_str());
  }

  if (merge_frequency_hz_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Merge frequency must be positive. Value is: %.2f", merge_frequency_hz_);
    throw std::invalid_argument("Merge frequency must be positive.");
  }

  auto merge_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / merge_frequency_hz_));
  merge_timer_ = this->create_wall_timer(
    merge_period,
    std::bind(&RadarObjectMerger::merge_and_publish, this));
}

void RadarObjectMerger::on_velocity(const Float32::ConstSharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(data_mutex_);
  current_velocity_kmph_ = msg->data;
  merge_all_radars_ = current_velocity_kmph_ < velocity_threshold_kmph_;
}

void RadarObjectMerger::on_radar_input(const DetectedObjects::ConstSharedPtr msg, const std::string & topic_name)
{
  std::lock_guard<std::recursive_mutex> lock(data_mutex_);
  radar_objects_[topic_name] = msg;
  if (!merge_all_radars_ && topic_name == front_center_radar_topic_) {

    if (msg->header.frame_id != merge_frame_) {
      RCLCPP_WARN(this->get_logger(), "Frame ID mismatch. Expected: %s, Received: %s", merge_frame_.c_str(), msg->header.frame_id.c_str());
      return;
    }

    objects_pub_->publish(*msg);
  }
}

void RadarObjectMerger::merge_and_publish()
{
  std::lock_guard<std::recursive_mutex> lock(data_mutex_);

  if (!merge_all_radars_) 
    return;

  DetectedObjects merged_objects;
  rclcpp::Time latest_header_stamp(0, 0, this->get_clock()->get_clock_type());

  for (const auto& radar_object : radar_objects_) {
    const std::string& topic_name = radar_object.first;
    const DetectedObjects::ConstSharedPtr& objects_ptr = radar_object.second;
    if (!objects_ptr) continue;

    if (objects_ptr->header.frame_id != merge_frame_) {
      RCLCPP_WARN(this->get_logger(), "Frame ID mismatch. Expected: %s, Received: %s", merge_frame_.c_str(), objects_ptr->header.frame_id.c_str());
      continue;;
    }

    rclcpp::Time current_header_stamp = objects_ptr->header.stamp;

    if ((last_merge_time_ - message_timeout_duration_).seconds() > current_header_stamp.seconds()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Message from %s is too old. Header time: %f",
        topic_name.c_str(), current_header_stamp.seconds());
      continue;
    }

    if (current_header_stamp > latest_header_stamp) {
      latest_header_stamp = current_header_stamp;
    }

    merged_objects.objects.insert(
      merged_objects.objects.end(),
      objects_ptr->objects.begin(),
      objects_ptr->objects.end()
    );
  }

  if (!merged_objects.objects.empty()) {
    merged_objects.header.stamp = latest_header_stamp;
    merged_objects.header.frame_id = merge_frame_;
    objects_pub_->publish(merged_objects);
    last_merge_time_ = merged_objects.header.stamp;
  }
}

}  // namespace radar_object_merger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_object_merger::RadarObjectMerger)

