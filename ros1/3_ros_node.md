# ROS Node

A ROS (Robot Operating System) node is a fundamental building block in ROS. Each node is a process that performs a specific task within a ROS system, and nodes communicate with each other by publishing and subscribing to **topics** (named channels carrying messages). Nodes can also interact through **services** or **actions**.

## Creating a ROS Node in Python

To create a ROS node in Python, you use the `rospy` library. Below is an example of how to create a simple ROS node.

### Example 1: Basic ROS Node in Python

1. In your `my_robot` directory, create a `scripts` folder and then create a Python file, for example, `simple_node.py`, with the following code:

    ```python
    #!/usr/bin/env python3
    import rospy

    if __name__ == '__main__':
        # Initialize the ROS node with a name
        rospy.init_node('my_simple_node')

        rospy.loginfo("Node starting...")

        rospy.sleep(1)

        rospy.loginfo("Node exiting...")
    ```

2. Make the Python script executable:

    ```bash
    chmod +x simple_node.py
    ```

3. Start the ROS master with `roscore` in a terminal, and then execute the Python file in another terminal:

    ```bash
    rosrun my_robot simple_node.py
    ```

### Example 2: ROS Node with Loop in Python

1. Create another Python file, for example, `loop_node.py`:

    ```python
    #!/usr/bin/env python3
    import rospy

    if __name__ == '__main__':
        # Initialize the ROS node
        rospy.init_node('my_loop_node')  # each node must have a unique name

        rospy.loginfo("Node starting...")

        rate = rospy.Rate(10)  # 10 Hz loop rate

        while not rospy.is_shutdown():
            rospy.loginfo("Hello, ROS!")
            rate.sleep()
    ```

2. Run the script with the ROS master running:

    ```bash
    rosrun my_robot loop_node.py
    ```

3. To check if the node is running, open a new terminal and use the following command:

    ```bash
    rosnode list
    ```

## Creating a ROS Node in C++

To create a ROS node in C++, you use the `roscpp` library. Below is a similar example of creating a ROS node in C++.

### Example 1: Basic ROS Node in C++

1. In your `my_robot` package, create a `src` directory if it doesn’t already exist. Create a C++ file, for example, `simple_node.cpp`, with the following code:

    ```cpp
    #include <ros/ros.h>

    int main(int argc, char** argv) {
        // Initialize the ROS node
        ros::init(argc, argv, "my_simple_node");
        ros::NodeHandle nh; // to start the node

        ROS_INFO("Node starting...");

        ros::Duration(1.0).sleep(); // Sleep for 1 second

        ROS_INFO("Node exiting...");

        return 0;
    }
    ```

2. In your `CMakeLists.txt` file, add the executable and link libraries:

    ```cmake
    add_executable(simple_node src/simple_node.cpp)
    target_link_libraries(simple_node ${catkin_LIBRARIES})
    ```

3. Build your package:

    ```bash
    catkin_make
    ```

4. Start `roscore` in one terminal, and then in another terminal, run the node:

You will find the executable file in `/devel/lib/my_robot/` directory

    ```bash
    rosrun my_robot simple_node
    ```

### Example 2: ROS Node with Loop in C++

1. Create another C++ file, for example, `loop_node.cpp`, in the `src` directory with the following code:

    ```cpp
    #include <ros/ros.h>

    int main(int argc, char** argv) {
        // Initialize the ROS node
        ros::init(argc, argv, "my_loop_node");
        ros::NodeHandle nh;

        ROS_INFO("Node starting...");

        ros::Rate rate(10);  // Set loop rate to 10 Hz

        while (ros::ok()) {
            ROS_INFO("Hello, ROS!");
            rate.sleep();
        }

        return 0;
    }
    ```

2. Add the executable to your `CMakeLists.txt`:

    ```cmake
    add_executable(loop_node src/loop_node.cpp)
    target_link_libraries(loop_node ${catkin_LIBRARIES})
    ```

3. Build the package again:

    ```bash
    catkin_make
    ```

4. Run the node with the ROS master running:

    ```bash
    rosrun my_robot loop_node
    ```

5. To check the active nodes, use:

    ```bash
    rosnode list
    ```

--- 

To get more info/kill/ping about the running node:

    ```bash
    rosnode info /node_name
    rosnode kill /node_name
    rosnode ping /node_name
    ```

To Visualize the ROS in a ghrph:

    ```bash
    rosrun rqt_graph rqt_graph
    ```