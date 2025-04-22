# Radar Object Merger

## Overview

This ROS 2 package provides the `radar_object_merger` node, which subscribes to object detections (`autoware_perception_msgs::msg::DetectedObjects`) from multiple radar sensors and merges them into a single output topic. The merging strategy dynamically adapts based on the ego vehicle's speed.

## Features

*   Merges `DetectedObjects` from a configurable list of radar topics.
*   Uses vehicle speed to switch between two merging strategies:
    *   **Low Speed (< `velocity_threshold_kmph`)**: Merges objects from *all* configured radars periodically based on `merge_frequency_hz`.
    *   **High Speed (>= `velocity_threshold_kmph`)**: Merges objects *only* from the `front_center_radar_topic` whenever a new message arrives on that topic (event-based).
*   Filters potentially stale messages based on a comparison involving `last_merge_time_` and `message_timeout_sec`.
*   Implemented as a C++ ROS 2 composable node component.


## Parameters

Parameters are typically loaded from `config/radar_object_merger.param.yaml` via the launch file.

*   `velocity_threshold_kmph` (double, default: 5.0): Speed threshold in km/h to switch between low-speed (timer-based) and high-speed (event-based) merging strategies.
*   `radar_topics` (list of strings, default: []): A list of all input radar `DetectedObjects` topic names to subscribe to.
*   `front_center_radar_topic` (string, default: ""): The specific topic name from `radar_topics` that corresponds to the front-center radar. This radar's data is always considered in low-speed mode and is the *only* data used (and the trigger) in high-speed mode. **Required for high-speed mode functionality.**
*   `merge_frequency_hz` (double, default: 10.0): Frequency (Hz) for timer-based merging in low-speed mode.
*   `message_timeout_sec` (double, default: 0.02): Timeout value (seconds) used in the message filtering condition `(last_merge_time_ - message_timeout_duration_).seconds() > current_header_stamp.seconds()`. Messages older than `last_merge_time_ - message_timeout_duration_` are considered stale.
*   `merge_frame` (string, default: "base_link"): Frame ID set in the header of the published merged `DetectedObjects` message.

## Topics

*   **Subscribed Topics:**
    *   `~/input/velocity` (`std_msgs::msg::Float32`): Ego vehicle velocity in km/h. Remapped in the launch file (e.g., from `/vehicle/status/velocity_kmph`).
    *   Topics listed in `radar_topics` (`autoware_perception_msgs::msg::DetectedObjects`): Input object detections from individual radars.
*   **Published Topics:**
    *   `~/output/objects` (`autoware_perception_msgs::msg::DetectedObjects`): Merged object detections. Remapped in the launch file (e.g., to `/perception/object_recognition/radar/merged_objects`).
