# ROS2 Topics and Node Communication

In ROS2, **topics** are named channels used to exchange messages between nodes. Nodes communicate by **publishing** (sending) messages on topics and **subscribing** (receiving) messages from topics. This publish-subscribe mechanism enables seamless communication between distributed nodes in the ROS2 network.

Each topic carries a specific type of message, defined by a **message type** (e.g., `std_msgs/msg/String` for text data or `sensor_msgs/msg/Image` for images).

## Creating a Publisher and Subscriber in Python

### Publisher Node in Python

Let's create a node that publishes messages on the topic `/greetings`.

1. Create a Python file named `publisher_node.py`:

    ```python
    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class GreetingsPublisher(Node):
        def __init__(self):
            super().__init__('greetings_publisher')
            self.publisher_ = self.create_publisher(String, '/greetings', 10)
            timer_period = 1.0  # seconds (1Hz)
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = f'Hello, ROS2! {self.i}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1

    def main(args=None):
        rclpy.init(args=args)
        publisher = GreetingsPublisher()
        rclpy.spin(publisher)
        publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2. Make the file executable:
    ```bash
    chmod +x publisher_node.py
    ```

3. Add the executable to `setup.py`:
    ```python
    entry_points={
        'console_scripts': [
            'publisher_node = my_package.publisher_node:main',
        ],
    }
    ```

4. Build and run:
    ```bash
    colcon build --packages-select my_package
    source install/setup.bash
    ros2 run my_package publisher_node
    ```

### Subscriber Node in Python

Now let's create a node that subscribes to the `/greetings` topic.

1. Create a Python file named `subscriber_node.py`:

    ```python
    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class GreetingsSubscriber(Node):
        def __init__(self):
            super().__init__('greetings_subscriber')
            self.subscription = self.create_subscription(
                String,
                '/greetings',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning

        def listener_callback(self, msg):
            self.get_logger().info(f'Received: "{msg.data}"')

    def main(args=None):
        rclpy.init(args=args)
        subscriber = GreetingsSubscriber()
        rclpy.spin(subscriber)
        subscriber.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2. Add to `setup.py` and build as before, then run:
    ```bash
    ros2 run my_package subscriber_node
    ```

## Creating a Publisher and Subscriber in C++

### Publisher Node in C++

1. Create `src/publisher_node.cpp`:

    ```cpp
    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"
    using namespace std::chrono_literals;

    class GreetingsPublisher : public rclcpp::Node {
    public:
        GreetingsPublisher() : Node("greetings_publisher"), count_(0) {
            publisher_ = this->create_publisher<std_msgs::msg::String>("/greetings", 10);
            timer_ = this->create_wall_timer(
                1000ms, std::bind(&GreetingsPublisher::timer_callback, this));
        }
    private:
        void timer_callback() {
            auto message = std_msgs::msg::String();
            message.data = "Hello, ROS2! " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
    };

    int main(int argc, char * argv[]) {
        rclpy::init(argc, argv);
        rclpy::spin(std::make_shared<GreetingsPublisher>());
        rclpy::shutdown();
        return 0;
    }
    ```

2. Add to `CMakeLists.txt`:
    ```cmake
    add_executable(publisher_node src/publisher_node.cpp)
    ament_target_dependencies(publisher_node rclcpp std_msgs)
    install(TARGETS publisher_node DESTINATION lib/${PROJECT_NAME})
    ```

### Subscriber Node in C++

1. Create `src/subscriber_node.cpp`:

    ```cpp
    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    class GreetingsSubscriber : public rclcpp::Node {
    public:
        GreetingsSubscriber() : Node("greetings_subscriber") {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "/greetings",
                10,
                std::bind(&GreetingsSubscriber::topic_callback, this, std::placeholders::_1));
        }
    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
            RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    };

    int main(int argc, char * argv[]) {
        rclpy::init(argc, argv);
        rclpy::spin(std::make_shared<GreetingsSubscriber>());
        rclpy::shutdown();
        return 0;
    }
    ```

2. Add to `CMakeLists.txt` similar to publisher.

## Topic Management Commands

```bash
# List all active topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /greetings

# Show topic info
ros2 topic info /greetings

# Show topic message type
ros2 topic type /greetings

# Publish a message manually
ros2 topic pub /greetings std_msgs/msg/String "{data: 'Hello from CLI'}"

# Visualize communication
ros2 run rqt_graph rqt_graph