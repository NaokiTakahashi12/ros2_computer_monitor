# ros2_computer_monitor

This is a package that publish the computer status with diagnostic_updater.

# TODO

1. Describe parameters using generate_parameter_library.
1. Create a node to publish storage occupancy, etc.

# Node

## cpu_monitor_node

This node publish a diagnostic topic based on the information in the following file.

+ /proc/stat
+ /proc/cpuinfo

### Publish topics

+ `/diagnostics` (diagnostic_msgs/msg/DiagnosticArray)

### Parameters

+ `cpu_usage_warn_threshold` (double, default_value: 60)
+ `cpu_usage_error_threshold` (double, default_value: 95)
+ `cpu_speed_warn_threshold` (double, default_value: 3)
+ `cpu_speed_error_threshold` (double, default_value: 4.5)
+ `diagnostic_period` (double, default_value: 1)

# Installation

## Requirements

ros2 humble or higher

## Dependent packages

+ diagnostic_msgs
+ diagnostic_updater

## build

```shell
$ cd <your colcon workspace>/src
$ git clone https://github.com/NaokiTakahashi12/ros2_computer_monitor.git
$ cd ..
$ colcon build
```

## Usage

CPU monitor

```shell
$ ros2 run ros2_computer_monitor cpu_monitor_node
```
