

Next Steps - 

Get motors finalized

SD Card implemented in setup, need to wire hardware.

Calculate Declination Angle

***   IMPORTANT
Check my wires for left motor. It spins in one direction but not the other 
indicating either the code isn;t setting IN4 LOW or HIGH or the wire is bad

DIRECTION  INPUT 1 INPUT 2 INPUT 3 INPUT 4
Forward 0 1 0 1
Backward  1 0 1 0
Right 0 1 0 0
Left  0 0 0 1
Stop  0 0 0 0

With my robot build, My "Right" motor is facing from the rear, which means the left motor runs in the opposite direction. 
This is my truth chart for Motor functionality

                                    Single Track Steer
                        Left Motor                      Right Motor
DIRECTION         INPUT 1         INPUT 2         INPUT 3         INPUT 4
Forward             0               1               1               0
Backward            1               0               0               1
Right               0               1               0               0
Left                0               0               1               0
Stop                0               0               0               0

                                    Dual Track Steer
                        Left Motor                      Right Motor
DIRECTION         INPUT 1         INPUT 2         INPUT 3         INPUT 4
Right               0               1               0               1
Left                1               0               1               0
