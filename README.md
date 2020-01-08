# Self Balancing Arduino Robot
This is the code to make a 2 wheel self balancing robot on Arduino (Variable names/comments are in french)

This code contains the algorithms necessary to read the accelerometer and gyroscope while correcting for drift.
To balance the robot, pwm is used to control the motors speed and pid is used to calculate the speed and direction at which
the motors should be spun.
