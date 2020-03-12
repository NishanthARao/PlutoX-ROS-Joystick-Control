# PlutoX-ROS-Joystick-Control
This is a simulation of the PlutoX drone in ROS-Melodic, gazebo9 environment, using a linux-supported Joystick.

![Image](https://github.com/NishanthARao/PlutoX-ROS-Joystick-Control/blob/master/Pluto.png)

You can watch a video demonstration here:
https://youtu.be/dHBQtf9umPo

This project is a simulation of the PlutoX drone created by DronaAviation, IIT Bombay. The model of the drone is created on blender, which is imported as a mesh file in a .xacro file that defines the quadcopter model. The mathematical model of the drone is written in a C++ file present in the ```src``` directory. The output is then calulated by a set of 9 PID algorithms, and the final state is published to Gazebo via the ```setModelState``` service. 

Additionally, a Joystick control has been added to navigate the world present in Gazebo. Upon pressing the R1 button on the Joystick, the drone automagically goes to the point (1,1,1) and hovers there. In this mode, the joystick controls are disabled. Upon repressing the R1 button, the position hold mode is disabled and the Joystick control is back! Please note that the R1 button in my Joystick can be mapped differently in the Joystick you are using. One way to identify which button corresponds to enabling the position hold, is by running the joystick node testing ROS program given in the tutorials. The button that corresponds to a change in the 6th element of the index is your position hold button.  

Note that the drone goes crazy when its far from (1,1,1) and the position hold button is pressed. This is because the PID algorithm saturates and acts crazy as the error between the current position and the desired position is large. 

# Installation
Make sure you have installed ROS, GAZEBO and their dependencies.
Additionally, you have to install the following packages:
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 
sudo apt-get update
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt-get install ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-position-controllers
```

1) **SETUP the Joystick.**

You need a linux-compatible joystick with ROS packages installed. Make sure to follow the tutorials to set up the Joystick from the official ROS documentation:
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

2) **Download and run the simulation**

Create a ROS workspace
```
mkdir -p ~/catkin_cpp_ws/src
cd ~/catkin_cpp_ws/src
catkin_init_workspace
cd ~/catkin_cpp_ws/
catkin_make
```
Add this workspace to your linux environment by sourcing the setup file to .bashrc. Assuming you are inside the home directory, 
```
cd ~
gedit .bashrc
```
Add this line at the end of the file.
```
source ~/catkin_cpp_ws/devel/setup.bash
```
Create a ROS package in your workspace. We will call it fly_bot_cpp. Add the dependencies.
```
cd ~/catkin_cpp_ws/src
catkin_create_pkg fly_bot_cpp roscpp rospy std_msgs
cd ~/catkin_cpp_ws/
catkin_make
```
Download all the files into your newly created folder 'fly_bot_cpp'. You have to replace the currently existing CMakeLists, package.xml, src folder and not MERGE!. Thus, the folder hierarchy must be as follows:
```
/catkin_cpp_ws/src/fly_bot_cpp
  -config
  -include
  -launch
  -meshes
  .
  .
  .
  -CMakeLists.txt
  -package.xml
  -urdf.rviz
```
After you have all the files, it is important that you have to compile the fly_bot_cpp package:
```
cd ~/catkin_cpp_ws/
catkin_make
```
**Note**:The program written in C++ for controlling the drone (kwad.cpp) is present in the ```src``` folder. When you run ```catkin_make```, this will automatically compile the C++ program (kwad.cpp) also (doesn't happen by default, happens when you include certain lines of code in the CMakeLists.txt). Thus, if you make any changes in kwad.cpp, make sure to run ```catkin_make``` again!

Make sure that you don't get any errors while running this command. This package was exclusively written for ROS Melodic only. In case you are using higher versions of ROS, please check the catkin_make documentation. Also, make sure to revisit all the instructions mentioned above.

Now, it is important that you have all the models required for generating the gazebo environment. For this, copy the folder ```models```. Go to home directory and press ```ctrl+h```. You should see some additional folders pop up, and one of them will be ```.gazebo```. Go inside this folder and paste the ```models``` folder here. The ```ctrl+h``` command brings up the hidden folders. So if they are annoying to see each time you open the home directory, just prese ```ctrl+h``` again, so that the hidden folders are now hidden.

After this, close the terminal. Open a new terminal and launch the Gazebo9 environment. Make sure that the Joystick is plugged in. You may get the errors like "couldn't force feedback" or "No pid gains mentioned..." and that's fine.
```
roslaunch fly_bot_cpp pluto_gazebo.launch
```
If you have followed all the instructions properly, the gazebo simulation must pop up, and you should see the PlutoX drone on a grass plane. Note that it may take some time for the gazebo simulation to launch, as we have just added some new models to it. Worst case, just restart your system and check again.

Once the simulation launches, open another terminal and type:
```
rosrun fly_bot_cpp kwad
```
You should see the propellers rotating. Enjoy navigating in the environment, and test if the position hold algorithm works!

**Tip** 

1)If you have a nVidia graphics card, you can make the simulation look cleaner with some extra effects like shadows. Just go to the ```pluto.world``` file present in the ```worlds``` directory of the package, and change the ```<shadows>false</shadows>``` line to ```<shadows>true</shadows>```

2)You can change the pilot camera's position to whatever you desire in the file ```pluto.world``` present in the ```worlds``` directory of the package and edit the line ```<xyz>0 -1.5 0.35</xyz>``` in the ```<gui>``` definition. Note that this is a 3D position vector relative to the DRONE and not the environment.
