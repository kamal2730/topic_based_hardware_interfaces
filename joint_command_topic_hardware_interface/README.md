# Joint Command Topic Based System

The Joint Command Topic Based System implements a ros2_control `hardware_interface::SystemInterface` supporting command and state interfaces through the ROS topic communication layer. It publishes commands as `control_msgs/JointCommand` messages, one message per interface type (position, velocity, effort), on a dedicated topic per interface type.

## ros2_control urdf tag

The `joint_command_topic_hardware_interface` has a few `ros2_control` urdf tags to customize its behavior.

### Parameters

* joint_commands_topic: (default: "/robot_joint_commands"). Base topic for the joint command topics. Example: `<param name="joint_commands_topic">/my_topic_joint_commands</param>`.
* joint_states_topic: (default: "/robot_joint_states"). Example: `<param name="joint_states_topic">/my_topic_joint_states</param>`.
* trigger_joint_command_threshold: (default: 1e-5). Used to avoid spamming the joint command topic when the difference between the current joint state and the joint command is smaller than this value, set to -1 to always send the joint command. Example: `<param name="trigger_joint_command_threshold">0.001</param>`.
* sum_wrapped_joint_states: (default: "false"). Used to track the total rotation for joint states the values reported on the `joint_states_topic` wrap from 2*pi to -2*pi when rotating in the positive direction. (Isaac Sim only reports joint states from 2*pi to -2*pi) Example: `<param name="sum_wrapped_joint_states">true</param>`.

### Per-joint Parameters

* mimic: Defined name of the joint to mimic. This is often used concept with parallel grippers. Example: `<param name="mimic">joint1</param>`.
* multiplier: Multiplier of values for mimicking joint defined in mimic parameter. Example: `<param name="multiplier">-2</param>`.

## Example

```xml
        <ros2_control name="name" type="system">
            <hardware>
              <plugin>joint_command_topic_hardware_interface/JointCommandTopicSystem</plugin>
              <param name="joint_commands_topic">/topic_based_joint_commands</param>
              <param name="joint_states_topic">/topic_based_joint_states</param>
            </hardware>
            <joint name="joint_1">
                <command_interface name="position"/>
                <command_interface name="velocity"/>
                <state_interface name="position">
                  <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            ...
        </ros2_control>
```

## Topics

On a `write()` call that passes the `trigger_joint_command_threshold` check, one `control_msgs/JointCommand` message is published per interface type to a topic derived from the `joint_commands_topic` parameter:

* `<joint_commands_topic>/position`: the joints whose `position` command is being driven.
* `<joint_commands_topic>/velocity`: the joints whose `velocity` command is being driven.
* `<joint_commands_topic>/effort`: the joints whose `effort` command is being driven.

Each message carries those joints in its `joint_names` field, the matching command values in `values`, and the interface type in `interface_name`. Subscribers must listen on the topic that matches the interface they are interested in, e.g. with the default parameter a joint with `position` and `velocity` command interfaces publishes to `/robot_joint_commands/position` and `/robot_joint_commands/velocity`.

A command interface that no active controller writes to holds `NaN`. Such a joint is left out of its message, and an interface type where no joint is driven at all is not published, so a subscriber never has to filter `NaN` out itself. Declaring a command interface therefore does not on its own guarantee traffic on the matching topic.

No message is published while the summed difference between the joint states and the joint commands stays at or below `trigger_joint_command_threshold`, which is the steady state once a controller has converged. Set the threshold to -1 to publish on every cycle.
