# Robotathon ESP32
----Alternative Movement----
There are two different movement styles. Test both and see which feels better overall. 
1. Left joystick->left motor (current)
   Right joystick->right motor
2. Left joystick->both motors (commented out)
   Right joystick-> adjusts one side depending on x position

---- Line Automation----
 If correction is negative, move left by adjusting the left wheel slower while adjusitng the right wheel faster
 if correction is positive, move right by adjusting the right wheel slower while adjusitng the left wheel faster
 if correction is zero, move straight forward at full speed
 I decided to not use the invert sensors function because it is easers to just use 1,000 for black and 0 for white
 as if i use it the other way i would have to invert everything and also i wouldn't know how that would be able to work becuase
 *right now the lineHelper function works by multiplying the sensor values by the weights but if its 1,000 in the middle and 0 on white
 then the error would go based off the middle which i dont think would be as effecient and just more complicated then using 0
 for white and 1,000 for black


----Color Automation----
How to control
1. Switch to color automation.
2. Put robot over sample color.
4. It will continue to move until it finds the color again.

----Wall Automation----
This automation part is heavily heavily heavily dependant on thresholds.
Before even attempting, you have to ensure multiple different things work together.
1. I created TIME_90_DEGREES.
   You need to debug and find how long these numbers need to be for the robot to actually rotate 90 degrees. There's no actual way to tell the robot to turn a certain degree it has to be done manually.
2. In the wallAutomation() method, there are several variables you need to test as well.
   a. wallThreshold: This is the number (cm) until the robot should treat it as a wall.
   b. rightThreshold: This is the number (cm) that tells you how far it is from the right wall.
   c. speedAdjust: This is how much the motor will slow down when it's readjusting within walls.
   1. if no wall in front, move forward.
   2. If wall in front, check right sensor, if open path, rotate 90 degrees clockwise.
   3. Else (wall on right), rotate 90 degrees counter clockwise.
           Algorithm A: While moving forward, it should stay within the right bounds. If it's 
               within these bounds, just move forward. It may still drift a bit, but that's okay.
               If it's minor, I'll leave as is. If it's major, then I'll implement PD control
               to adjust the motors based off previous value. If it is out of these bounds, then
               adjust motors based off set values (not changing).
           Algorithm B: Just move forward without worrying about centering. If rotate90CW/CCW is
               implemented well enough, then no adjustments should be necessary.
           Algorithm C: Combination of A and B. Move forward normally, but before turning 90 degrees, 
               if previous right distance > current right distance, (getting closer to wall), then
               adjust delay for TIME_90_DEGREES so that it turns for longer. If previous right
               distance < current right distance, (getting farther away from wall), then adjust
               delay for TIME_90_DEGREES so that it turns for less time.
           Notes: Calculate the connection between delay time and angle turned. Use that to adjust
               TIME_90_DEGREES based off previous and current. Calculate previous and current by
               and make them a set time apart TIME_90_DEGREES. You can then use these two to create 
               a triangle and figure out the angle it's currently moving at. From there, you can 
               calculate how much to turn to be parallel to the wall after a turn. This assumes it 
               never hits the wall which should stand true, if not there's an error in the logic. 
               The delta time is TIME_90_DEGREES because you use this to turn 90 degrees, so after 
               calculating the angle you can easily figure out how much you need to adjust to stay 
               parallel.

----Notes----
2. Adjust color offset delay so robot is over the color. 
4. I use TOP_MOTOR_SPEED for a lot of things. Don't change this variable but make another one if you think it's too fast.
5. Calculate size of wheel and radius and create equation to automatically see how long it takes to do 90 degrees.
6. Design robot so it keeps turning towards the right and then it stops when its close enough (threshold).
7. Remove left sensor integration.