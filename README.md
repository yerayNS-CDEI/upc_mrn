This is a ros2 package for the practices of the subject Mobile Robotics and Navigation (MRN) of the Master in Automation and Robotics (MUAR) of the Universitat Politècnica de Catalunya (UPC).

These practices are performed on Turtlebot4 platforms simulated and real.

#Dependencies

For these practices you will need a PC with Ubuntu 22.04 and ros humble installed (ros-humble-desktop-full recommended).

Apart from ROS humble, the following dependencies are required for the practices. They all can be installed via apt:
- ssh
- git
- python3-colcon-common-extensions
- ignition-fortress
- ros-humble-rplidar-ros 
- ros-humble-rqt-tf-tree
- ros-humble-teleop-twist-keyboard
- ros-humble-turtlebot4-simulator 
- ros-humble-turtlebot4-desktop 
- ros-humble-turtlebot4-navigation 
- ros-humble-turtlebot4-node 
- ros-humble-turtlebot4-tutorials

##Setup script

For the students convenience, we provide an script to install the dependencies, configure a ros workspace (~/ros2_ws) and clone this package, in your personal laptop (Ubuntu 22.04 and ros humble required).
This script needs to be executed once. 

1. Download the script:

`wget --no-check-certificate -O mrn_setup.sh http://bit.ly/upc_mrn_laptop`

Terminal output should say it downloaded the file successfully, otherwise, try again. 

2. Then, execute the script (your password will be required):

`source mrn_setup.sh`

Check that no errors occurred and then you can remove the script and the logs that were generated (rm mrn_setup.sh log_mrn_setup.log).

#Simulation

