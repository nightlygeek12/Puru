This is a tutorial to run the Puru Bot

There is two folder in this repo.

a) Puru joy -This is the folder which contains the joystick code for puru. Put this folder in your pc "catkin_ws/src" folder. -Run "catkin_make"

b)Puru Portenta Wheel Odom -This is the arduino IDE file for the portenta microcontroller. Place this folder in Arduino sketch folder if you want to modify the code.

To run the Puru bot

1) Make sure you have the puru_joystick in your "catkin/ws/src" folder and run "catkin_make".
2) Then ON the switch on the robot. DO NOT RELEASE THE EMERGENCY button.
3) Connect the LAN cable to the PC
4) Open a new terminal and run "roslaunch rosserial_server socket.launch".
5) Open another terminal and run "rostopic list" to see if the topics are available.
6) Then, open a new terminal and run "roslaunch puru_joystick joy_puru.launch".
7) Release the Emergency button.
8) Then push the left joystick to make the robot go forward/reverse. The right joystick controls the robots turn.
