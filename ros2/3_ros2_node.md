# ROS2 Node

A ROS2 node is a fundamental building block in ROS2. Each node is a process that performs a specific task within a ROS2 system, and nodes communicate with each other by publishing and subscribing to **topics** (named channels carrying messages). Nodes can also interact through **services**, **actions**, or **parameters**.

## Creating a ROS2 Node in Python

To create a ROS2 node in Python, you use the `rclpy` library. Below are examples of creating ROS2 nodes.

### Example 1: Basic ROS2 Node in Python

1. In your `my_package` package, create a `scripts` folder and then create `simple_node.py`:

    ```python
    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node

    def main(args=None):
        rclpy.init(args=args)
        node = Node('my_simple_node')
        
        node.get_logger().info("Node starting...")
        rclpy.spin_once(node, timeout_sec=1.0) # to keep the node working
        node.get_logger().info("Node exiting...")
        
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2. Make it executable:
    ```bash
    chmod +x simple_node.py
    ```

3. Add the executable to `setup.py` (for Python packages):
    ```python
    entry_points={
        'console_scripts': [
            'simple_node = my_package.simple_node:main',
        ],
    }
    ```

4. Build and run:
    ```bash
    colcon build --packages-select my_package
    source install/setup.bash
    ros2 run my_package simple_node
    ```

### Example 2: ROS2 Node with OOP

1. Create `loop_node.py`:
    ```python
    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node

    class LoopNode(Node):
        def __init__(self):
            super().__init__('my_loop_node')
            self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
            
        def timer_callback(self):
            self.get_logger().info('Hello, ROS2!')

    def main(args=None):
        rclpy.init(args=args)
        node = LoopNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2. Run it:
    ```bash
    ros2 run my_package loop_node
    ```

## Creating a ROS2 Node in C++

### Example 1: Basic ROS2 Node in C++

1. Create `src/simple_node.cpp`:
    ```cpp
    #include "rclcpp/rclcpp.hpp"

    int main(int argc, char ** argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<rclcpp::Node>("my_simple_node");
        
        RCLCPP_INFO(node->get_logger(), "Node starting...");
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(node->get_logger(), "Node exiting...");
        
        rclcpp::shutdown();
        return 0;
    }
    ```

2. Add to `CMakeLists.txt`:
    ```cmake
    add_executable(simple_node src/simple_node.cpp)
    ament_target_dependencies(simple_node rclcpp)
    install(TARGETS simple_node DESTINATION lib/${PROJECT_NAME})
    ```

3. Build and run:
    ```bash
    colcon build --packages-select my_package
    ros2 run my_package simple_node
    ```

### Example 2: ROS2 Node with Loop in C++

1. Create `src/loop_node.cpp`:
    ```cpp
    #include "rclcpp/rclcpp.hpp"

    class LoopNode : public rclcpp::Node {
    public:
        LoopNode() : Node("my_loop_node") {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&LoopNode::timer_callback, this));
        }
    private:
        void timer_callback() {
            RCLCPP_INFO(this->get_logger(), "Hello, ROS2!");
        }
        rclcpp::TimerBase::SharedPtr timer_;
    };

    int main(int argc, char ** argv) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<LoopNode>());
        rclcpp::shutdown();
        return 0;
    }
    ```

2. Add to `CMakeLists.txt` and build as before.

## Node Management Commands

```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /node_name

# Remapping node name at runtime
ros2 run my_package my_node --ros-args --remap __node:=new_node_name

# Visualize nodes
rqt_graph
ros2 run rqt_graph rqt_graph