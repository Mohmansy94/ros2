## Create a catkin workspace:
To create a catkin workspace, follow these steps:

1. Open a terminal and navigate to the desired location where you want to create the workspace. or simply do this in the home directory

2. Run the following command to create the workspace directory:
    ```
    mkdir -p ~/catkin_ws/src
    ```

3. Navigate to the catkin_ws workspace:
    ```
    cd ~/catkin_ws/
    ```

4. Build the workspace:
    ```
    catkin_make
    ```
You can start adding your ROS packages inside the `src` directory of the workspace. After finishing the development inside `src` directory, go back to catking workspace and run `catkin_make` again.

5. Source the workspace:
    ```
    source ~/catkin_ws/devel/setup.bash
    ```
We can add `/opt/noetic/setup.bash` to end of the `/.bashre`


## Create a catkin package:

To create a catkin package and add it to the workspace, follow these steps:

1. Navigate to the `src` directory of your catkin workspace:
    ```
    cd ~/catkin_ws/src
    ```

2. Run the following command to create a new package:
    ```
    catkin_create_pkg my_package std_msgs rospy roscpp
    ```
    Replace `my_package` with the desired name of your package (my_robot). You can also specify additional dependencies after `my_package` if needed.

3. Navigate back to the catkin workspace:
    ```
    cd ~/catkin_ws
    ```

4. Build the workspace to include the new package:
    ```
    catkin_make
    ```

5. Source the workspace:
    ```
    source ~/catkin_ws/devel/setup.bash
    ```
    This ensures that the new package is available in your ROS environment.

You can now start developing your ROS package inside the `src/my_package` directory. Remember to run `catkin_make` again after making any changes to your package.
