sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyACM0
roslaunch simulation_env Base.launch
roslaunch simulation_env amcl.launch
roslaucnh lab6_a_star lab6.launch
