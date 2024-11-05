# ROS1 

## ROS1 (Noetic) Installation

Requirements:
Ubuntu 20.04 / Python3 (recommended)

Installation Steps:

Here are the installation steps for ROS1 Noetic on Ubuntu 20.04 with Python3:

1. Set up your sources.list:
    ```
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    ```

2. Set up your keys:
    ```
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

    ```

3. Update your package index:
    ```
    sudo apt update
    ```

4. Install ROS1 Noetic:
    ```
    sudo apt install ros-noetic-desktop-full
    ```

5. Set up the environment:
    ```
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

6. Install additional dependencies:
    ```
    sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    ```

7. Initialize rosdep:
    ```
    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update
    ```
