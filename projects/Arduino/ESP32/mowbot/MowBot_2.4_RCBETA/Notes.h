

Next Steps - 

Calculate Declination Angle

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


WayPoint Arrays

float latWayPoint[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
float lonWayPoint[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}

Waypoint #5 latitude = latWayPoint[4] 
Waypoint #7 longitude = lonWayPoint[6]
