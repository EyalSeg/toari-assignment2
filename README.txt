this package is our submission for assigmnemts 2&3.

HOW TO USE
launch the robot (gazeo/real robot)
then:
1. rosrun ass2 kinect_proc.py
2. rosrun ass2 Controller.py

the Controller will require input as defined in the assignemt. the following commands are supported:
1 - will move ~1 meter (if no obstacles are directly infront of the robot)
2 - will rotate in place a gien amount of degrees.
3 - will return the distance to the largest red object in the frame - ASSUMES THERE IS ONE
4 - will send the arm towards the largest red object in the frame - ASSUMES THERE IS ONE
