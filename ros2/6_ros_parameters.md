# ROS2 Parameters

Parameters in ROS2 allow you to configure nodes at runtime without changing code. They're ideal for settings that may need adjustment between launches or deployments.

## Parameter Basics

Parameters are key-value pairs that can be:
- Set at node startup
- Modified during runtime
- Accessed by other nodes
- Saved to/loaded from files

Common use cases:
- Configuration values (e.g., thresholds, rates)
- Tuning parameters (e.g., PID gains)
- Operational modes

## Declaring and Using Parameters

### Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParamDemoNode(Node):
    def __init__(self):
        super().__init__('param_demo_node')
        
        # Declare parameters with default values
        self.declare_parameter('my_string', 'hello')
        self.declare_parameter('my_int', 42)
        self.declare_parameter('my_double', 3.14)
        self.declare_parameter('my_bool', True)
        
        # Get parameter values
        my_string = self.get_parameter('my_string').value
        my_int = self.get_parameter('my_int').value
        
        self.get_logger().info(f"String param: {my_string}")
        self.get_logger().info(f"Int param: {my_int}")

def main(args=None):
    rclpy.init(args=args)
    node = ParamDemoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```
### C++ Example

```cpp
#include "rclcpp/rclcpp.hpp"

class ParamDemoNode : public rclcpp::Node {
public:
    ParamDemoNode() : Node("param_demo_node") {
        // Declare parameters
        this->declare_parameter<std::string>("my_string", "hello");
        this->declare_parameter<int>("my_int", 42);
        this->declare_parameter<double>("my_double", 3.14);
        this->declare_parameter<bool>("my_bool", true);
        
        // Get parameters
        std::string my_string = this->get_parameter("my_string").as_string();
        int my_int = this->get_parameter("my_int").as_int();
        
        RCLCPP_INFO(this->get_logger(), "String param: %s", my_string.c_str());
        RCLCPP_INFO(this->get_logger(), "Int param: %d", my_int);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamDemoNode>());
    rclcpp::shutdown();
    return 0;
}
```
## Parameter Callbacks

Register a callback to handle parameter changes at runtime:

### Python Callback

```python
from rcl_interfaces.msg import ParameterDescriptor

class ParamDemoNode(Node):
    def __init__(self):
        super().__init__('param_demo_node')
        
        # Parameter descriptor (optional metadata)
        desc = ParameterDescriptor(description='A sample integer parameter')
        
        self.declare_parameter('my_int', 42, desc)
        
        # Add callback
        self.add_on_set_parameters_callback(self.param_change_callback)
    
    def param_change_callback(self, params):
        for param in params:
            self.get_logger().info(
                f"Parameter {param.name} changed to {param.value}")
        return rclpy.node.SetParametersResult(successful=True)

```
### C++ Callback
 ```cpp
 #include "rcl_interfaces/msg/parameter_descriptor.hpp"

class ParamDemoNode : public rclcpp::Node {
public:
    ParamDemoNode() : Node("param_demo_node") {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "A sample integer parameter";
        
        this->declare_parameter<int>("my_int", 42, desc);
        
        // Add callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ParamDemoNode::param_change_callback, this, std::placeholders::_1));
    }

private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    rcl_interfaces::msg::SetParametersResult param_change_callback(
        const std::vector<rclcpp::Parameter> &params) {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        
        for (const auto &param : params) {
            RCLCPP_INFO(this->get_logger(), 
                "Parameter %s changed to %s",
                param.get_name().c_str(),
                param.value_to_string().c_str());
        }
        return result;
    }
};
```
## Using YAML Parameter Files

Create a YAML file (`config/params.yaml`):

```yaml
param_demo_node:
  ros__parameters:
    my_string: "hello_world"
    my_int: 99
    my_double: 1.618
    my_bool: false
```
Load parameters at launch:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='param_demo_node',
            name='param_demo_node',
            parameters=['config/params.yaml']
        )
    ])
```
Or load from command line:

```bash
ros2 run my_package param_demo_node --ros-args --params-file config/params.yaml
```

## Parameter CLI Commands

```bash
# List all parameters
ros2 param list

# Get parameter value
ros2 param get /param_demo_node my_int

# Set parameter
ros2 param set /param_demo_node my_string "new_value"

# Dump parameters to file
ros2 param dump /param_demo_node > saved_params.yaml

# Load parameters from file
ros2 run my_package param_demo_node --ros-args --params-file saved_params.yaml
```