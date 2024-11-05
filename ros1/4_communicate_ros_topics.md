# ROS Topics and Node Communication

In ROS, **topics** are named channels used to exchange messages between nodes. Nodes communicate by **publishing** (sending) messages on topics, and **subscribing** (receiving) messages from topics. This publish-subscribe mechanism enables seamless communication between distributed nodes in the ROS network.

When a node publishes to a topic, all other nodes subscribed to that topic receive the messages. Each topic carries a specific type of message, defined by a **message type** (e.g., `std_msgs/String` for text data or `sensor_msgs/Image` for images).

## Creating a Publisher and Subscriber in Python

### Publisher Node in Python

Let's create a node that publishes messages on the topic `/greetings`.

1. Create a Python file named `publisher_node.py`:

    ```python
    #!/usr/bin/env python3
    import rospy
    from std_msgs.msg import String

    def publisher():
        # Initialize the ROS node
        rospy.init_node('greetings_publisher', anonymous=True) # anonymous to run the same node several times without any problem.

        # Define the publisher on the topic `/greetings` with message type `String`
        pub = rospy.Publisher('/greetings', String, queue_size=10)

        rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            message = String()
            message = "Hello, ROS!"
            rospy.loginfo(f"Publishing: {message}")
            pub.publish(message)
            rate.sleep()

        rospy.loginfo("STOP")
    if __name__ == '__main__':
        try:
            publisher()
        except rospy.ROSInterruptException:
            pass
    ```

2. Make the file executable:

    ```bash
    chmod +x publisher_node.py
    ```

3. Run `roscore` in one terminal, and in another terminal, run the publisher node:

    ```bash
    rosrun my_robot publisher_node.py
    ```

### Subscriber Node in Python

Now let's create a node that subscribes to the `/greetings` topic and receives messages.

1. Create a Python file named `subscriber_node.py`:

    ```python
    #!/usr/bin/env python3
    import rospy
    from std_msgs.msg import String

    def callback(msg):
        rospy.loginfo(f"Received message: {msg.data}")

    def subscriber():
        # Initialize the ROS node
        rospy.init_node('greetings_subscriber', anonymous=True)

        # Define the subscriber to the `/greetings` topic with message type `String`
        rospy.Subscriber('/greetings', String, callback)

        rospy.spin()  # Keeps the node active and listening

    if __name__ == '__main__':
        try:
            subscriber()
        except rospy.ROSInterruptException:
            pass
    ```

2. Make the file executable:

    ```bash
    chmod +x subscriber_node.py
    ```

3. In another terminal, run the subscriber node:

    ```bash
    rosrun my_robot subscriber_node.py
    ```

4. Observe the subscriber terminal for messages received from the publisher.

---

## Creating a Publisher and Subscriber in C++

### Publisher Node in C++

1. Create a C++ file named `publisher_node.cpp` in the `src` directory of your package:

    ```cpp
    #include <ros/ros.h>
    #include <std_msgs/String.h>

    int main(int argc, char** argv) {
        // Initialize the ROS node
        ros::init(argc, argv, "greetings_publisher"); // we can add "ros::init_options::AnonymousName"
        ros::NodeHandle nh;

        // Define the publisher on the `/greetings` topic
        ros::Publisher pub = nh.advertise<std_msgs::String>("/greetings", 10);

        ros::Rate rate(1);  // 1 Hz

        while (ros::ok()) {
            std_msgs::String msg;
            msg.data = "Hello, ROS!";
            ROS_INFO("Publishing: %s", msg.data.c_str());
            pub.publish(msg);
            rate.sleep();
        }

        return 0;
    }
    ```

2. In your `CMakeLists.txt` file, add the publisher node executable:

    ```cmake
    add_executable(publisher_node src/publisher_node.cpp)
    target_link_libraries(publisher_node ${catkin_LIBRARIES})
    ```

3. Build your package:

    ```bash
    catkin_make
    ```

4. Run `roscore` in one terminal, and in another terminal, run the publisher node:

    ```bash
    rosrun my_robot publisher_node
    ```

### Subscriber Node in C++

1. Create a C++ file named `subscriber_node.cpp` in the `src` directory of your package:

    ```cpp
    #include <ros/ros.h>
    #include <std_msgs/String.h>

    void callback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("Received message: %s", msg->data.c_str());
    }

    int main(int argc, char** argv) {
        // Initialize the ROS node
        ros::init(argc, argv, "greetings_subscriber");
        ros::NodeHandle nh;

        // Define the subscriber to the `/greetings` topic
        ros::Subscriber sub = nh.subscribe("/greetings", 10, callback);

        ros::spin();  // Keeps the node active and listening

        return 0;
    }
    ```

2. In your `CMakeLists.txt` file, add the subscriber node executable:

    ```cmake
    add_executable(subscriber_node src/subscriber_node.cpp)
    target_link_libraries(subscriber_node ${catkin_LIBRARIES})
    ```

3. Build your package again:

    ```bash
    catkin_make
    ```

4. In another terminal, run the subscriber node:

    ```bash
    rosrun my_robot subscriber_node
    ```

5. Observe the subscriber terminal for messages received from the publisher.

---

