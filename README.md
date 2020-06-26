# MultiDroneSimulation
## Introduction
  This project for simulating three drone and let them fly in a specific path autonomously over set of sensors located in the 
ground. The sensor nodes should able to pass messege to drone when a drone comes into it's region. Here I am simulate the world in gazebo environment and all my underlying program to control the drone and sensor operation is written in c++.


I have added 3 drones here each drone is devided the areas between them and assign to sensors in that region. Here I have print the reading from sensor node into terminal. I am maintaining three terminals for three drones.



## This how drone aproches thier sensor nodes
![](images/drone_aproach_sensor_clear.gif)

## This how drone read messeges from sensor nodes
![](images/sample_read_message.gif)

#### to setup in the debian/ubuntu  environment following must be followed.
* Gazebo(recomended Gazebo9 or later )

* Install Ros melodic (follow the steps [here](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html#rosgazebo))
  - this installation contain gazebo9 itself no need to install yourself
  
* Install Mavros library (follow the steps [here](https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html#binary-installation-debian--ubuntu)) 
  - this link contains the binary installation(recommended) if you planing to edit the source follow the step [here](https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html#source-installation)
  
* clone the latest px4 Firmware (follow the steps [here](https://dev.px4.io/master/en/simulation/multi_vehicle_simulation_gazebo.html#build-and-test )) 

###### if all above result satisfied your machine the clone this repo and launch it
