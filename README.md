# radar_object_merger

A ROS2 component node for merging detected objects from multiple radar sources, with support for velocity-based switching between all-radar merging and event-based publishing from a front-center radar.

## Features

- Subscribes to multiple radar topics (list of `autoware_auto_perception_msgs/msg/DetectedObjects`).
- Subscribes to a velocity topic (`std_msgs/msg/Float32`).
- Publishes merged objects to `~/output/objects`.
- Switches between merging all radars (low speed) and publishing only the front-center radar (high speed), based on a velocity threshold.
- Configurable message timeout and merge frequency.

## Parameters

| Name                      | Type                | Default      | Description                                                                 |
|---------------------------|---------------------|--------------|-----------------------------------------------------------------------------|
| `velocity_threshold_kmph` | `double`            | `5.0`        | Velocity threshold (km/h) for switching between all-radar and front-only.   |
| `radar_topics`            | `list of strings`   | `[]`         | List of radar topics to subscribe to.                                       |
| `front_center_radar_topic`| `string`            | `""`         | Topic name for the front-center radar.                                      |
| `merge_frequency_hz`      | `double`            | `10.0`       | Frequency (Hz) for periodic merging.                                        |
| `message_timeout_sec`     | `double`            | `0.02`       | Timeout (seconds) for radar messages to be considered valid.                |
| `merge_frame`             | `string`            | `base_link`  | Frame ID for the merged output.                                             |

## Topics

### Subscribed

- `~/input/velocity` (`std_msgs/msg/Float32`): Current vehicle velocity in km/h.
- Each topic in `radar_topics` (`autoware_auto_perception_msgs/msg/DetectedObjects`): Radar object detections.

### Published

- `~/output/objects` (`autoware_auto_perception_msgs/msg/DetectedObjects`): Merged radar objects.

## Behavior

- When velocity is below `velocity_threshold_kmph`, merges all radar topics and publishes at `merge_frequency_hz`.
- When velocity is above the threshold, publishes only the front-center radar detections as they arrive.
- Ignores radar messages older than `message_timeout_sec` relative to the last merge time.

## Usage Example

```yaml
radar_object_merger:
  ros__parameters:
    velocity_threshold_kmph: 5.0
    radar_topics:
      - "/radar/left/objects"
      - "/radar/right/objects"
      - "/radar/front_center/objects"
    front_center_radar_topic: "/radar/front_center/objects"
    merge_frequency_hz: 10.0
    message_timeout_sec: 0.02
    merge_frame: "base_link"
```

## Notes

- If `radar_topics` is empty, the node will not process any radar data.
- If `front_center_radar_topic` is not set or not in `radar_topics`, high-speed mode will not function as intended.
- All radar topics must publish `DetectedObjects` messages.

## License

See repository for license information.
