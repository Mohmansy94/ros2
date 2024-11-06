# ROS Services

In ROS, **services** provide a synchronous, request-response communication mechanism between nodes, similar to a function call. Unlike topics (which use a publish-subscribe model), services allow one node (the **client**) to send a request to another node (the **server**) and wait for a response. Services are used for tasks that need confirmation or response, such as changing a robot's mode, obtaining sensor data on-demand, or executing a specific command.

## Service Architecture

1. **Service Server**: This node advertises a service and waits to handle requests from clients. It processes each request, performs an action, and sends back a response.
2. **Service Client**: This node sends requests to a service server and waits for a response.

A service is defined by a **service type**, which includes a request and response structure. For example, `std_srvs/SetBool` is a service type that has a boolean `request` (true/false) and a boolean `response` indicating success.

## Creating a Service in Python

### Service Definition

1. Define a custom service in a `.srv` file (optional). If using a standard service like `std_srvs/SetBool`, skip this step.
2. If creating a custom service, define the service file in your package's `srv` folder, for example:

    ```plaintext
    my_robot/srv/AddTwoInts.srv
    ```

    ```plaintext
    int64 a
    int64 b
    ---
    int64 sum
    ```

3. Update your `CMakeLists.txt` and `package.xml` to add the custom service.

### Service Server Node in Python

Here’s a Python example of a service server that adds two integers. If you’re using the standard `std_srvs/SetBool` service, you can skip defining a custom service.

1. Create a file `add_two_ints_server.py`:

    ```python
    #!/usr/bin/env python
    import rospy
    from my_robot.srv import AddTwoInts, AddTwoIntsResponse

    def handle_add_two_ints(req):
        rospy.loginfo(f"Adding {req.a} + {req.b}")
        return AddTwoIntsResponse(req.a + req.b)

    def add_two_ints_server():
        rospy.init_node('add_two_ints_server')
        s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
        rospy.loginfo("Service 'add_two_ints' is ready.")
        rospy.spin()

    if __name__ == "__main__":
        add_two_ints_server()
    ```

2. Make the file executable:

    ```bash
    chmod +x add_two_ints_server.py
    ```

3. Start the service server:

    ```bash
    rosrun my_robot add_two_ints_server.py
    ```

### Service Client Node in Python

1. Create a file `add_two_ints_client.py`:

    ```python
    #!/usr/bin/env python
    import rospy
    from my_robot.srv import AddTwoInts, AddTwoIntsRequest

    def add_two_ints_client(a, b):
        rospy.wait_for_service('add_two_ints')
        try:
            add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
            response = add_two_ints(a, b)
            return response.sum
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    if __name__ == "__main__":
        rospy.init_node('add_two_ints_client')
        result = add_two_ints_client(3, 5)
        rospy.loginfo(f"Result: {result}")
    ```

2. Make the file executable:

    ```bash
    chmod +x add_two_ints_client.py
    ```

3. Start the client:

    ```bash
    rosrun my_robot add_two_ints_client.py
    ```

---

## Creating a Service in C++

### Service Server Node in C++

1. Create a file `add_two_ints_server.cpp` in the `src` directory of your package:

    ```cpp
    #include <ros/ros.h>
    #include "my_robot/AddTwoInts.h"

    bool add(my_robot::AddTwoInts::Request &req, my_robot::AddTwoInts::Response &res) {
        res.sum = req.a + req.b;
        ROS_INFO("Request: a=%ld, b=%ld", (long int)req.a, (long int)req.b);
        ROS_INFO("Sending response: [%ld]", (long int)res.sum);
        return true;
    }

    int main(int argc, char **argv) {
        ros::init(argc, argv, "add_two_ints_server");
        ros::NodeHandle nh;

        ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
        ROS_INFO("Service 'add_two_ints' is ready.");
        ros::spin();

        return 0;
    }
    ```

2. Add the server node executable in `CMakeLists.txt`:

    ```cmake
    add_executable(add_two_ints_server src/add_two_ints_server.cpp)
    target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
    ```

3. Build your package:

    ```bash
    catkin_make
    ```

4. Run the service server:

    ```bash
    rosrun my_robot add_two_ints_server
    ```

### Service Client Node in C++

1. Create a file `add_two_ints_client.cpp` in the `src` directory of your package:

    ```cpp
    #include <ros/ros.h>
    #include "my_robot/AddTwoInts.h"

    int main(int argc, char **argv) {
        ros::init(argc, argv, "add_two_ints_client");
        ros::NodeHandle nh;

        ros::ServiceClient client = nh.serviceClient<my_robot::AddTwoInts>("add_two_ints");
        my_robot::AddTwoInts srv;
        srv.request.a = 3;
        srv.request.b = 5;

        if (client.call(srv)) {
            ROS_INFO("Sum: %ld", (long int)srv.response.sum);
        } else {
            ROS_ERROR("Failed to call service add_two_ints");
            return 1;
        }

        return 0;
    }
    ```

2. Add the client node executable in `CMakeLists.txt`:

    ```cmake
    add_executable(add_two_ints_client src/add_two_ints_client.cpp)
    target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
    ```

3. Build the package again:

    ```bash
    catkin_make
    ```

4. Run the service client:

    ```bash
    rosrun my_robot add_two_ints_client
    ```

---