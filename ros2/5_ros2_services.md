# ROS2 Services

In ROS2, **services** provide a synchronous, request-response communication mechanism between nodes. Unlike topics (which use publish-subscribe), services allow a **client** node to send a request to a **server** node and wait for a response. Services are ideal for on-demand operations like configuration changes, calculations, or triggering actions.

## Service Architecture

1. **Service Server**: Advertises a service and handles incoming requests
2. **Service Client**: Calls the service and waits for a response

Services are defined by a **service type** which specifies the request and response structure (e.g., `example_interfaces/srv/AddTwoInts`).

## Creating a Service in Python

### Service Definition

1. For custom services, create a `.srv` file in your package's `srv` directory:

    ```plaintext
    # my_package/srv/AddTwoInts.srv
    int64 a
    int64 b
    ---
    int64 sum
    ```

2. Update `package.xml`:
    ```xml
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```

3. Update `CMakeLists.txt`:
    ```cmake
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME}
      "srv/AddTwoInts.srv"
    )
    ```

### Service Server Node in Python

1. Create `add_two_ints_server.py`:

    ```python
    #!/usr/bin/env python3
    from example_interfaces.srv import AddTwoInts
    import rclpy
    from rclpy.node import Node

    class AddTwoIntsServer(Node):
        def __init__(self):
            super().__init__('add_two_ints_server')
            self.srv = self.create_service(
                AddTwoInts,
                'add_two_ints',
                self.add_two_ints_callback)

        def add_two_ints_callback(self, request, response):
            response.sum = request.a + request.b
            self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
            return response

    def main(args=None):
        rclpy.init(args=args)
        server = AddTwoIntsServer()
        rclpy.spin(server)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2. Add to `setup.py`:
    ```python
    entry_points={
        'console_scripts': [
            'add_two_ints_server = my_package.add_two_ints_server:main',
        ],
    }
    ```

### Service Client Node in Python

1. Create `add_two_ints_client.py`:

    ```python
    #!/usr/bin/env python3
    import sys
    from example_interfaces.srv import AddTwoInts
    import rclpy
    from rclpy.node import Node

    class AddTwoIntsClient(Node):
        def __init__(self):
            super().__init__('add_two_ints_client')
            self.cli = self.create_client(AddTwoInts, 'add_two_ints')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.req = AddTwoInts.Request()

        def send_request(self, a, b):
            self.req.a = a
            self.req.b = b
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()

    def main(args=None):
        rclpy.init(args=args)
        client = AddTwoIntsClient()
        response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
        client.get_logger().info(f'Result: {response.sum}')
        client.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

## Creating a Service in C++

### Service Server Node in C++

1. Create `src/add_two_ints_server.cpp`:

    ```cpp
    #include "example_interfaces/srv/add_two_ints.hpp"
    #include "rclcpp/rclcpp.hpp"
    using std::placeholders::_1;
    using std::placeholders::_2;

    class AddTwoIntsServer : public rclcpp::Node {
    public:
        AddTwoIntsServer() : Node("add_two_ints_server") {
            service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                "add_two_ints",
                std::bind(&AddTwoIntsServer::add, this, _1, _2));
        }
    private:
        void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
            response->sum = request->a + request->b;
            RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld b: %ld",
                        request->a, request->b);
        }
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
    };

    int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<AddTwoIntsServer>());
        rclcpp::shutdown();
        return 0;
    }
    ```

2. Add to `CMakeLists.txt`:
    ```cmake
    add_executable(add_two_ints_server src/add_two_ints_server.cpp)
    ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)
    install(TARGETS add_two_ints_server DESTINATION lib/${PROJECT_NAME})
    ```

### Service Client Node in C++

1. Create `src/add_two_ints_client.cpp`:

    ```cpp
    #include "example_interfaces/srv/add_two_ints.hpp"
    #include "rclcpp/rclcpp.hpp"
    using namespace std::chrono_literals;

    class AddTwoIntsClient : public rclcpp::Node {
    public:
        AddTwoIntsClient() : Node("add_two_ints_client") {
            client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        }

        bool send_request(int a, int b) {
            while (!client_->wait_for_service(1s)) {
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
            }
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;
            auto result = client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(
                    this->get_node_base_interface(), result) == 
                    rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Sum: %ld", result.get()->sum);
                return true;
            }
            return false;
        }
    private:
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    };

    int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        auto client = std::make_shared<AddTwoIntsClient>();
        client->send_request(3, 5);
        rclcpp::shutdown();
        return 0;
    }
    ```

## Service Management Commands

```bash
# List all services
ros2 service list

# Find service type
ros2 service type /add_two_ints

# Call a service from CLI
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"

# Show service info
ros2 service info /add_two_ints