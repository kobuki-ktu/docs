# Yujin Kobuki ROS2 setup documentation

Author: Ugnius Stašaitis

# Short description
This document covers installing and setting up ROS2 on a RaspberryPI
(the Kobuki robot) and a more powerful desktop PC/VM/Distrobox machine.
It also covers acquiring, compiling and configuring ROS2 packages for
the Kobuki which allow the robot to be teleoperated and/or controlled via a
lidar-powered navigation stack.

## 1. Setting up the base system(s)

### 1. Installing Ubuntu.
Since this guide is using ROS Jazzy, you will need to install `Ubuntu 24.04.3 LTS (Noble)`.
#### 1. On the Kobuki
1. Download [`RaspberryPi Imager`](https://www.raspberrypi.com/software/)
2. Select `The Ubuntu server LTS 64-bit` image.
3. Use the
  [***super secret menu***](https://www.clustered-pi.com/blog/raspberry-pi-imager-secret-menu.html)
  to configure the WIFI and SSH access.
4. Flash the image
5. Insert the SD card into the PI and power it up!

  **A note on SSH:**
      I highly recommend using SSH to configure the RaspberryPI you'll be using to
      control the Kobuki.
      If you're not famillilar with SSH and how to set it up, a useful article can be
      found [here](https://www.digitalocean.com/community/tutorials/ssh-essentials-working-with-ssh-servers-clients-and-keys).

#### 2. Setting up another machine
You will need another machine for ROS capabilities which the RaspberryPI is not
powerful enough to properly run (eg.: Rviz2, compiling heavy ROS packages)

You have 4 options for this (ordered from simplest and easiest to hardest and most advanced):

1. If you have a spare machine, you can simply install Ubuntu on it.
2. You can install Ubuntu on a spare partition and dual-boot.

3. Set up a virtual machine (*this is the method I used*)
  - If you're running Linux, I recommend using
    [`Virtual Machine Manager`](https://virt-manager.org/).
  - If you're running Windows - there's
    [`VirtualBox`](https://www.virtualbox.org/).

  In either case, you will need to set up bridged networking on the VM in order
  for the VM to be able to communicate with the RaspberryPI. This process is
  moderately complex on Linux and I cannot comment how easy/hard it is to do
  on Windows.

4. Using containers
  - You can run [`ROS on Docker`](https://blog.robotair.io/the-complete-beginners-guide-to-using-docker-for-ros-2-deployment-2025-edition-0f259ca8b378). *Note: this works really well, but needs a lot of setup.*
  - If you're running Linux, [`Distrobox`](https://distrobox.it/)
  can let you run a containerized Ubuntu instance on your distro of choice.
  - If you're running Windows, you can experiment using the Windows Subsystem
  for Linux, but I have no idea how well it will work for this use case.

*Or, of course, you could simply bully ChatGPT to help you set it up*

## 2. Installing ROS2

### 1. On the Kobuki:

```
sudo apt -y update && sudo apt -y install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt -y install software-properties-common
sudo add-apt-repository -y universe

sudo apt -y update && sudo apt -y install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt -y update
sudo apt -y upgrade

sudo apt -y install ros-dev-tools
sudo apt -y install ros-jazzy-ros-base

sudo apt -y install ros-jazzy-demo-nodes-cpp

source /opt/ros/jazzy/setup.bash
LINE='source /opt/ros/jazzy/setup.bash'
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc
```

### 2. On the Desktop

```
sudo apt -y update && sudo apt -y install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt -y install software-properties-common
sudo add-apt-repository -y universe

sudo apt -y update && sudo apt -y install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt -y update
sudo apt -y upgrade

sudo apt -y install ros-dev-tools
sudo apt -y install ros-jazzy-ros-desktop

source /opt/ros/jazzy/setup.bash
LINE='source /opt/ros/jazzy/setup.bash'
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc
```

## 3. Testing the base ROS2 installation and communication
Ensure both machines are connected to the same network.

On the Kobuki run:

```
ros2 run demo_nodes_cpp talker
```

On the Desktop run:

```
ros2 run demo_nodes_cpp listener
```

If everything is set up and working correctly, you will see the Kobuki
transmitting "Hello World" messages and the Desktop machine succesfully
receiving them.

## 4. Installing the Kobuki packages on ROS2

Install a `udev` rule for the Kobuki, so it always appears as `/dev/kobuki` when
connected to the RaspberryPI (instead of `/dev/ttyUSBx`).
```
wget https://raw.githubusercontent.com/kobuki-ktu/kobuki_ftdi/devel/60-kobuki.rules
sudo cp 60-kobuki.rules /etc/udev/rules.d
sudo service udev reload
sudo service udev restart
```

The Kobuki ROS packages are not available on the ROS Jazzy repos, as such, we
will have to build them from source.

The packages depend on ECL, however, a few of the required ones are/were
unavailable on the ROS jazzy repos, so we must also build them from source.

The Kobuki and ECL source code is available on my GitHub fork.
The fork also has a few patches applied, since the original ECL code
needs a few small fixes in order to compile on current C++ compilers.
```
mkdir -p ~/kobuki/src
cd ~/kobuki

git clone https://github.com/kobuki-ktu/ecl_lite.git ~/kobuki/src/ecl_lite
git clone https://github.com/kobuki-ktu/ecl_core.git ~/kobuki/src/ecl_core

# ECL dependencies
sudo apt install -y ros-jazzy-angles
sudo apt install -y ros-jazzy-diagnostic-updater
sudo apt install -y ros-jazzy-ecl-build
sudo apt install -y ros-jazzy-sophus

colcon build
```

**TIP:** If a package fails to build, you can try to build the packages by groups:

`colcon build --packages-select-regex ecl`

Or even one by one:

`colcon build --packages-select-regex ecl_time`

If ECL was compiled succesfully, we can now compile the Kobuki packages:
```
mkdir -p ~/kobuki/src
cd ~/kobuki

git clone https://github.com/kobuki-ktu/kobuki_core.git ~/kobuki/src/kobuki_core
git clone https://github.com/kobuki-ktu/kobuki_ros_interfaces.git ~/kobuki/src/kobuki_ros_interfaces
git clone https://github.com/kobuki-ktu/kobuki_ros.git ~/kobuki/src/kobuki_ros
git clone https://github.com/kobuki-ktu/kobuki_velocity_smoother.git ~/kobuki/src/kobuki_velocity_smoother

colcon build
```

After installing the packages, you will need to activate your ROS2 overlay
(environment) each time you want to use these packages:
```
source ~/kobuki/install/setup.bash
```
**TIP**: Add this command to your [`.bashrc`](https://www.digitalocean.com/community/tutorials/bashrc-file-in-linux)

If everything built correctly, try connecting the Kobuki to the RaspberryPI and
running this command:
```
kobuki-simple-keyop
```
You should be able to control the robot via keyboard input

*Note: this is only testing the base Kobuki driver, which is communicating
with the robot directly - not through ROS2*

## 5. Note on launch files for the Kobuki packages

As the Kobuki project is no longer maintained,
many of the original launch files for the Kobuki ROS2 nodes need small to
moderate ammounts of fixup in order to work properly. As such, commands like:
```
ros2 launch kobuki_random_walker kobuki_random_walker_app.launch
```
are not going to work directly since they're using the original launch files

If you'll want to build more complex behaviour, you'll also need to edit the
launch files in order to configure packages and connect them together.

***You should always make a copy of the launch files instead of editing them
directly***

In every single ROS package you compiled via `colcon`, the launch files will
be located in
```
~/kobuki/install/<package_name>/share/<package_name>/launch
```

In every single ROS package you install via `apt install`, the launch files will
be located in
```
/opt/ros/jazzy/share/<package_name>/launch
```

## 6. Configuring keyboard control via ROS2

Keyboard control via ROS2 will require you to install a package for sending
keyboard commands to ROS2.

```
sudo apt install -y ros-jazzy-teleop-twist-keyboard
```

The next steps will require you to launch multiple terminals or use `tmux`

In one terminal run:
```
ros2 launch kobuki_node kobuki_node-launch.py
```
This connects the Kobuki driver to ROS and exposes various ROS topics for
controlling the robot and getting information out of it.

You can see these topics by running this command in another terminal
```
ros2 topic list
```
**Note:** the `kobuki_node` must be running for you to see the topic list.

The output will look something like this:
```
<...>
/commands/controller_info
/commands/digital_output
/commands/external_power
/commands/led1
/commands/led2
/commands/motor_power
/commands/reset_odometry
/commands/sound
/commands/velocity
/controller_info
/debug/raw_control_command
/debug/raw_data_command
/debug/raw_data_stream
/diagnostics
/events/bumper
/events/button
/events/cliff
/events/digital_input
/events/power_system
/events/robot_state
/events/wheel_drop
<...>

```

We are interested in the `/commands/velocity` topic if we want to move the robot.

If you run
```
ros2 topic info /commands/velocity
```

You will see the type of ROS messages it accepts, the number of ROS nodes
which are publishing to it, and the number of nodes which are subscribed to it.

```
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1
```

In this case the subscriber is the `kobuki_node` which is listening for
`geometry_msgs/msg/Twist` messages.

Lets send some of these messages!


This command runs another ROS node which will send messages
to `/commands/velocity` (launch it in another terminal)
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args --remap cmd_vel:=/commands/velocity

```

**Note:**
We use the `--remap cmd_vel:=/commands/velocity` because by default this
package outputs messages to the `/cmd_vel` topic, but the Kobuki is
listening on `/commands/velocity`. This redirects the messages to the correct
destination. It would also be valid to remap the kobuki_node's input from
`/commands/velocity` to `/cmd_vel` - it would also serve the same purpose.

Once you've launched the keyboard node, try moving the robot.

You can observe what values the are sent to `/commands/velocity` by calling this
command in another terminal.
```
ros2 topic echo /commands/velocity
```

If you're using the recommended 2 machine setup and the other machine is
connected to the same network as the Kobuki and ROS2 is installed correctly,
you can control the Robot from the other machine!

Simply run ONLY the `kobuki_node` on the Kobuki, and
ONLY the `teleop_twist_node` on the other machine.

## 7. Setting up LIDAR, SLAM and using RVIZ

## 8. Muxing inputs

## 9. Bringing up the Nav2 stack

## 10. Connecting an Xbox controller

## 11. Useful links
https://docs.nav2.org/setup_guides/index.html
https://wiki.ros.org/kobuki/Tutorials/Kobuki's%20Control%20System
https://idorobotics.com/2024/02/20/ros2-on-kobuki-turtlebot/
https://kobuki.readthedocs.io/en/devel/software.html
https://wiki.ros.org/yocs_velocity_smoother/indigo
