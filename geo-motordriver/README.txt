This is a Teensy 3.1 with an I2C DAC. It listens for serial commands
(from a Moteino) and drives two 36v brushless motor controllers (via
the DAC). It has direct connections to the throttle (via the DAC); the
brake; a reverse line; and the 3 hall effect sensors in the motors.

At the moment it's driven in "dumb" mode. The sensors are
ignored. There are hard-coded constants for minimum and maximum values
for each motor, and their speeds have been hand-tuned for my
motors. Yours will be different. Mine may be different tomorrow, for
that matter.

My goal is to build out the sensor support to make this self-correcting.

