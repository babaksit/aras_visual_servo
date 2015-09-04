INTRODUCTION
---------------

The aim of visual servoing is tracking an object by the feedback information of the vision sensor. There are different techniques for controlling the system, in this work, Image based visual servoing has been chosen. 
The goal of propsed system is tracking the featureless object by the kernel and moment measurement.
The configuration of the system is eye-in-hand. 


OVERVIEW
--------------

The main packages are:

aras_visual_servo_camera
aras_visual_servo_controller
aras_visual_servo_gazebo

---------------
How to run th code:

roslaunch aras_visual_servo_gazebo aras_visual_servo_gazebo.launch

roslaunch aras_visual_servo_gazebo aras_visual_servo_controller.launch

rosrun aras_visual_servo_camera aras_visual_servo_camera_node

rosrun aras_visual_servo_controller aras_visual_servo_controller_node 

AUTHORS
---------------

Babak Sistani Zadeh Aghdam <br>
Javad Ramezanzadeh <br>
Parisa Masnadi <br>
Ebrahim Abedloo <br>
Hamid D Taghirad (supervisor) <br>




