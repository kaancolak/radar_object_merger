#include "radar_object_merger/radar_object_merger.hpp"

#include <chrono>
#include <functional> // for std::bind
#include <string>     // for std::stod, std::string
#include <stdexcept>  // for std::invalid_argument, std::out_of_range
#include <iostream>   // For std::cout

namespace radar_object_merger
{

using std::placeholders::_1;
using std::placeholders::_2;

RadarObjectMerger::RadarObjectMerger(const rclcpp::NodeOptions & options)
: Node("radar_object_merger", options),
  last_merge_time_(0, 0, this->get_clock()->get_clock_type()),
  message_timeout_duration_(0, 0)
{

  velocity_threshold_kmph_ = this->declare_parameter<double>("velocity_threshold_kmph", 5.0);
  radar_topics_ = this->declare_parameter<std::vector<std::string>>("radar_topics", std::vector<std::string>());
  front_center_radar_topic_ = this->declare_parameter<std::string>("front_center_radar_topic", "");
  merge_frequency_hz_ = this->declare_parameter<double>("merge_frequency_hz", 10.0); // Store frequency
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
      bool front_center_found = false;
      for(const auto& topic : radar_topics_) {
          if (topic == front_center_radar_topic_) {
              front_center_found = true;
              break;
          }
      }
      if (!front_center_found) {
          RCLCPP_WARN(this->get_logger(), "Front center topic '%s' not found in 'radar_topics'.", front_center_radar_topic_.c_str());
      }
  }

  // Publisher
  merged_objects_pub_ = this->create_publisher<DetectedObjects>("~/output/objects", rclcpp::QoS(10));

  // Subscribers
  velocity_sub_ = this->create_subscription<Float32>(
    "~/input/velocity", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), std::bind(&RadarObjectMerger::on_velocity, this, _1));

  for (const auto & topic_name : radar_topics_) {
    auto sub = this->create_subscription<DetectedObjects>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this, topic_name](const DetectedObjects::ConstSharedPtr msg) {
        this->on_radar_input(msg, topic_name);
      });
    radar_subs_.push_back(sub);
    RCLCPP_INFO(this->get_logger(), "Subscribing to radar topic: %s", topic_name.c_str());
  }

  // --- Timer Setup ---
  if (merge_frequency_hz_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Merge frequency must be positive. Value is: %.2f", merge_frequency_hz_);
      throw std::invalid_argument("Merge frequency must be positive.");
  }
  merge_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / merge_frequency_hz_));

  // Create the timer initially (assuming starting at low speed or stopped)
  create_merge_timer();
  use_timer_based_merge_ = true; // Explicitly set initial state

  RCLCPP_INFO(this->get_logger(), "RadarObjectMerger node initialized. Starting in timer-based mode.");
}

void RadarObjectMerger::create_merge_timer()
{
    // Ensure timer is only created if not already active or if switching modes
    if (!merge_timer_ || !merge_timer_->is_canceled()) {
        cancel_merge_timer(); // Cancel existing timer just in case
    }
    merge_timer_ = this->create_wall_timer(
        merge_period_,
        std::bind(&RadarObjectMerger::merge_and_publish, this));
    RCLCPP_INFO(this->get_logger(), "Merge timer created/recreated (Rate: %.2f Hz).", merge_frequency_hz_);
}

void RadarObjectMerger::cancel_merge_timer()
{
    if (merge_timer_ && !merge_timer_->is_canceled()) {
        merge_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Merge timer cancelled.");
    }
}

void RadarObjectMerger::on_velocity(const Float32::ConstSharedPtr msg)
{
  // Lock needed as we modify current_velocity_kmph_ and potentially timer state
  std::lock_guard<std::recursive_mutex> lock(data_mutex_);
  current_velocity_kmph_ = msg->data;

  bool should_use_timer = current_velocity_kmph_ < velocity_threshold_kmph_;

  if (should_use_timer != use_timer_based_merge_) {
    use_timer_based_merge_ = should_use_timer;
    if (use_timer_based_merge_) {
      // Switched to low speed: Ensure timer is active
      RCLCPP_INFO(this->get_logger(), "Low speed detected (%.2f km/h < %.2f km/h). Switching to timer-based merging.", current_velocity_kmph_, velocity_threshold_kmph_);
      create_merge_timer(); // Create or recreate the timer
    } else {
      // Switched to high speed: Cancel timer
      RCLCPP_INFO(this->get_logger(), "High speed detected (%.2f km/h >= %.2f km/h). Switching to event-based merging (on front radar).", current_velocity_kmph_, velocity_threshold_kmph_);
      cancel_merge_timer();
    }
  }
}

void RadarObjectMerger::on_radar_input(const DetectedObjects::ConstSharedPtr msg, const std::string & topic_name)
{
    // Lock needed to update latest_radar_objects_ and potentially trigger merge
    std::lock_guard<std::recursive_mutex> lock(data_mutex_);
    latest_radar_objects_[topic_name] = msg;

    // If in high-speed mode and this is the front center radar, trigger merge
    if (!use_timer_based_merge_ && topic_name == front_center_radar_topic_) {
        merge_and_publish(); // Directly call merge logic
    }
}

void RadarObjectMerger::merge_and_publish()
{
    // Lock needed to access shared data (velocity, latest objects, last merge time)
    std::lock_guard<std::recursive_mutex> lock(data_mutex_);
    DetectedObjects merged_objects;

    rclcpp::Time latest_header_stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    // This variable determines if non-front radars are *allowed* based on speed,
    // but the actual inclusion depends on the merging mode (timer vs event).
    bool allow_other_radars = current_velocity_kmph_ < velocity_threshold_kmph_;

    for (const auto& pair : latest_radar_objects_) {
        const std::string& topic_name = pair.first;
        const DetectedObjects::ConstSharedPtr& objects_ptr = pair.second;

        if (!objects_ptr) continue;
        rclcpp::Time current_header_stamp = objects_ptr->header.stamp;

        bool is_front_center = (topic_name == front_center_radar_topic_);

        // Determine if this radar should be included based on current mode and speed rules
        bool include_this_radar = false;
        if (use_timer_based_merge_) { // Low speed mode (timer triggered)
            include_this_radar = (is_front_center || allow_other_radars);
        } else { // High speed mode (event triggered)
            include_this_radar = is_front_center; // Only include front center
        }

        if (include_this_radar) {
            if ( (last_merge_time_ - message_timeout_duration_).seconds() > current_header_stamp.seconds() ) {
                 RCLCPP_WARN_THROTTLE(
                     this->get_logger(), *this->get_clock(), 5000,
                     "Message from %s is too old. Header time: %f",
                     topic_name.c_str(), current_header_stamp.seconds());
                 continue;
            } else {
                 RCLCPP_INFO(
                     this->get_logger(),
                     "Message from %s is valid. Header time: %f",
                     topic_name.c_str(), current_header_stamp.seconds());
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
    }

    if (!merged_objects.objects.empty()) {
        merged_objects.header.stamp = latest_header_stamp;
        merged_objects.header.frame_id = merge_frame_;
        merged_objects_pub_->publish(merged_objects);

        last_merge_time_ = merged_objects.header.stamp;

        RCLCPP_INFO(
            this->get_logger(),
            "Merged %zu objects from %zu radars at time %f",
            merged_objects.objects.size(), latest_radar_objects_.size(),
            latest_header_stamp.seconds());
    }
}

}  // namespace radar_object_merger

// Register the component with class_loader
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_object_merger::RadarObjectMerger)
