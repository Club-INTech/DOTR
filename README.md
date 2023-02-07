# DOTR

***DOTR stands for "Drone over the ring"***. It is an end of the studies project that aims to control drone autonomously guiding it through the gates detected with its camera. This repository contains a **fork of a ros-gazebo** powered simulator, **tello code**, **gate detection code**, **navigation code** and **rc control code**. It also contains a **launcher** and instructions allowing to install all dependencies and run some predefined or even custom modes.

## Project architecture (dev)

## Install and run

There are 2 possible install modes: ***drone-only*** and ***drone+simulator***. The particularity of ***drone+simulator*** mode is that you must install **gazebo**, **ros2/galactic** and some **plugins** to use a simulator.

***Note: The following steps can only be applied to Linux***

---

### ***Clone the repository***

Clone the repository and create python virtual environment. You need python3.8 or greater to be installed.

```bash
    $ git clone git@github.com:Club-INTech/DOTR.git
    $ cd DOTR
    $ python3 -m venv venv
```

---

### ***Drone-only mode install***

The drone-only mode allows you to control the real tello edu drone and to receive frames from its camera. It also provides an autonomous mode in which drone detects gates and automatically crosses them. With this mode you can create ***empty, basic and tello video and orders providers, but ros providers will always be empty***.

```bash
    $ source venv/bin/activate
    $ cd drone_over_the_ring
    $ pip3 install -r requirements.txt
```

---

### ***Drone+simulator mode install***

First you need to install **ros2/galactic**, **gazebo** and some **ros-gazebo** plugins. For simplicity ros and gazebo install tutorials commands were copied here. If you want to follow the original ros install tutorial click [here](https://docs.ros.org/en/galactic/Installation.html).

In order to install **ros2/galactic** follow next steps.

Check you locale for UTF-8 support:

```bash
    $ locale  # check for UTF-8
    $ sudo apt update && sudo apt install locales
    $ sudo locale-gen en_US en_US.UTF-8
    $ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    $ export LANG=en_US.UTF-8
    $ locale  # verify settings
```

Ensure that Ubuntu Universe repository is enabled:

```bash
    $ sudo apt install software-properties-common
    $ sudo add-apt-repository universe
```

Add ros2 GPG key:

```bash
    $ sudo apt update && sudo apt install curl
    $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add ros2 repository to source list:

```bash
    $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install desktop **ros2/galactic**:

```bash
    $ sudo apt update
    $ sudo apt upgrade
    $ sudo apt install ros-galactic-desktop
    $ sudo apt install ros-dev-tools
```

You have installed **ros2/galactic**. Don't add its **setup.bash** script to your **bashrc** as we want to use it with virtual environment.

Install **gazebo**:

```bash
    
```

export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:${PWD}/src/tello-ros2-gazebo/tello_ros/tello_gazebo