<p align="center">
  <h2 align="center">User manual for Reforestabot </h2>

  <p align="justify">
  This user manual offers detailed instructions for using Reforestabot, a robotic platform created for the Universidad de las Américas Puebla (UDLAP) LRT4102: Design of Robotic Systems course. The installation procedure, requirements, and usage guidelines for the system's Arduino and Raspberry Pi components are covered in full in the guide. It also describes the teleoperation instructions that may be used to control the robot's movement, drilling, and seed delivery, among other tasks. Those with a basic understanding of robotics and programming concepts, as well as access to the necessary software and hardware, are the intended users for this guide.
	  
  <br>Universidad de las Américas Puebla (UDLAP) - Guided by professor Dr. César Martínez Torres. "https://www.linkedin.com/in/c%C3%A9sar-martinez-torres-617b5347/?originalSubdomain=mx>" 
  </p>
</p>
<be>

![Gif1](https://github.com/AldoPenaGa/LRT4102-Reforestabot/blob/main/Pictures/Gif1.gif)
![Gif2](https://github.com/AldoPenaGa/LRT4102-Reforestabot/blob/main/Pictures/Gif2.gif)

## Table of contents
- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Contributors](#contributors)

<div align= "justify">

### Introduction
Automating the reforestation process is the ultimate objective of the robotic platform Reforestabot. The robot has a movement system for moving around the terrain, a seed distribution system for planting seeds, and a drilling device for making holes in the ground. A Raspberry Pi computer and an Arduino microcontroller work together to control the robot using the standard Robot Operating System (ROS). The robot's motors and sensors are controlled at a low level by the Arduino, while higher level decision-making and communication functions are provided by the Raspberry Pi. The robot's motions and activities can be remotely controlled by the user using a teleoperation interface that allows for remote operation using a Linux-based computer.

### Prerequisites
It is assumed that a platform similar to Reforestabot is available, which constitutes the following:

1. Three Polulu 37D Gearmotor. 
2. An electro-pneumatic cylinder operative a 12 VDC.
3. Generic drill motor operative at 12 VDC and running at least at 22000 RPM.
4. Power supply capable of providing 4A or more at 12 VDC.
5. L289N or stronger H-Bridges operative at 2A or more.
6. Arduino MEGA (other models may function as long as they have PWM pins).
7. Raspberry Pi Model 4B.
8. Power bank for supplying the Raspberry and the Arduino, at least 2A or 2000 mA.
9. MicroSD Card with a capacity of at least 32 GB.

A computer with Linux Ubuntu installed is recommended (Ubuntu 20.04 was used), also it must have Arduino IDE and a code editor such as Visual Studio is necessary.

### Installation

The installation process consists of two key components, the Arduino and the Raspberry, both necessary for the correct function of the program.

**Arduino**

If Ardunio IDE has not been installed, a process for the installation can be found in the following link: https://www.geeksforgeeks.org/how-to-install-arduino-ide-on-ubuntu/

Once it has been installed, just open the IDE and select the proper `board` and `port` found in the `Tools` option located in the toolbar of the IDE. Then, paste the code of the `arduinoPart.ino` found in the Arduino folder of this repository.

Click on `verify` to check if it compilates correctly and then `upload` it to the board.

**Raspberry Pi**

This process is not as easy as the Arduino part, here, it is necessary to download the `Raspberry Imager Tool` found in the following link: https://www.raspberrypi.com/software/ . 

1. Insert the MicroSD card into the computer.
2. Open the `Raspberry Imager Tool`.
3. Select the intended device in the `CHOOSE DEVICE` option (in this case it is Raspberry Pi 4).
4. In the `CHOOSE OS` option select `Other general-purpose OS` then select `Ubuntu`, next select `Ubuntu Server 20.04.5 LTS (64.bit)` which was the operating system tested.
5. At `CHOOSE STORAGE` pick the MicroSD card elected. **Note that this will cause the MicroSD card to lose all its content, so before this process is completed review if any important data should be backed up**.
6. In the advanced section, we highly recommend enabling `SSH` so we can connect remotely to the Raspberry.
7. Double check if a Wifi connection has been set, if not, it is highly advisable to write the `SSID` and `password` here, otherwise, it will be necessary to do so manually.

Once the Ubuntu has been installed into the Raspberry, the next step is to install ROS too. Power up the Raspberry and connect a monitor via HDMI and a keyboard via USB, review if the Wifi was correctly set, in case it didn't was properyl set, try the following solution: 

1. Type `cd ..` twice in the terminal.
2. Type `sudo nano /etc/netplan/50-cloud-init.yaml`
3. Then in the text editor opened, type the following:

```
# This file is generated from information provided by
# the datasource.  Changes to it will not persist across an instance.
# To disable cloud-init's network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
    version: 2
    ethernets:
        eth0:
            optional: true
            dhcp4: true
    # add wifi setup information here ...
    wifis:
        wlan0:
            optional: true
            access-points:
                "YOUR-SSID-NAME":
                    password: "YOUR-NETWORK-PASSWORD"
            dhcp4: true
```
Change `"YOUR-SSID-NAME"` for the name of your Wifi (SSID) and `"YOUR-NETWORK-PASSWORD"` for the password of that SSID, the **indentation is pretty important**, so be careful with it.

Once all of this has been set proceed with the SSH enabling:

1. Type in the terminal: `sudo apt update` and then `sudo apt upgrade`
2. Once this has been done, proceed with `sudo apt-get install net-tools`
3. Now get the IP Address with iwconfig and remember it or write it somewhere.
4. Open your Ubuntu-based computer and type `ssh "user"@"IP-address"` replace the "user" for the user set in the Ubuntu configuration and the "IP-address" with one of the previous step. 

Next, the installation of ROS:

1. Open the terminal and type: `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
2. Then type: `sudo apt install curl` and then `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
3. Run a `sudo apt update`
4. And type `sudo apt install ros-noetic-ros-base`
5. Finally, `source /opt/ros/noetic/setup.bash`

The following step is to create a catkin workspace:

Type in the terminal `mkdir -p ~/catkin_ws/src`, then `cd ~/catkin_ws/` and compile it with `catkin_make`.

Finally, the ROS package needs to be uploaded to the Raspberry:
1. Type `cd catkin_ws` in the terminal.
2. Write then: `catkin_create_pkg Reforestabot roscpp rospy std_msgs`
3. Then, in a known location outside the catkin workspace clone the repository: `git clone https://github.com/AldoPenaGa/LRT4102-Reforestabot.git`.
4. Copy the content from the folder called ROS_package into the Reforestabot package we have just created, this is located in `cd ~/catkin_ws/src/Reforestabot` (command mv can be used `mv "pathSourceDirectory" "pathTargetDirectory"`).
5. Finally, `cd ~/catkin_ws` and `catkin_make`.

If everything has been properly installed, then here ends the installation section for the Raspberry. For further information please check: https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment


### Usage
The usage is pretty simple:

1. Open three SSH sessions from the terminal in the computer.
2. Run `roscore` in one session.
3. In the next session type `rosrun rosserial_python serial_node.py /dev/ttyACM0`. If a problem is presented, then type `sudo chmod 666 /dev/ttyACM0` before.
4. Finally, in the third SSH session, go to `cd catkin_ws/src/Reforestabot/src` and type `python3 teleopReforestabot.py`.

Typing any of the next keys will tell the Arduino to send the PWM signals to the robot components so they can do any of the actions the teleoperator is instructing:

```
    'w': Forward.
    'a': Left.
    'd': Right.
    's': Stop.
    'e': Small turn right.
    'q': Small turn left.
    'u': Lift electro-pneumatic cylinder.
    'j': Lower electro-pneumatic cylinder.
    'k': Stop electro-pneumatic cylinder.
    'h': Stop electro-pneumatic cylinder.
    'p': Drill.
    'l': Stop the drill.
    'm': Activate seeder.
    'n': Stop seeder.
```


### Contributors

| Name                          | Github                               |
|-------------------------------|--------------------------------------|
| Aldo Oziel Peña Gamboa        | https://github.com/AldoPenaGa        |
| Charbel Breydy Torres         | https://github.com/Buly1601          |
| Joan Carlos Monfil Huitle     | https://github.com/JoanCarlosMonfilHuitle|
| Enrique Rocha Espinoza        | https://github.com/Enrique-Rocha-Espinoza|

