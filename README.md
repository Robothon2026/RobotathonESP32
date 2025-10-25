# Robotathon ESP32

----WiFi connection----
Steps to connect to WiFi (not tested)
1. Change SSID and PASSWORD on the top of the file
2. Uncomment wifiConnectionSetup() and wifiConnection()
3. ctrl+f and replace "Console" with "telnetClient"
4. Find ip of esp32 after connection via the console. (this will change for different wifi connections)
4. Windows: enable telnet client in settings, open terminal and enter telnet {ip} 23
   Linux/mac: open terminal and enter telnet {ip} 23

----Alternative Movement----
There are two different movement styles. Test both and see which feels better overall. 
1. Left joystick->left motor (current)
   Right joystick->right motor
2. Left joystick->both motors (commented out)
   Right joystick-> adjusts one side depending on x position

----Color Automation----
How to control
1. Switch to color automation.
2. Put robot over sample color.
3. Click right trigger. 
4. It will continue to move until it finds the color again.

----Wall Automation----
This automation part is heavily heavily heavily dependant on thresholds.
Before even attempting, you have to ensure multiple different things work together.
1. I created two global variables TIME_90_DEGREES and TIME_180_DEGREES.
   You need to debug and find how long these numbers need to be for the robot to actually rotate 90 degrees. There's no actual way to tell the robot to turn a certain degree it has to be done manually. Once you find TIME_90_DEGREES, the 180 degrees should work since it's just * 2.
2. In the wallAutomation() method, there are several variables you need to test as well.
   a. turnThreshold: This is the number (cm) until the robot should treat it as a wall.
   b. leftRightThreshold: This is the number (cm) that tells you when it should start adjusting 
      within the walls. Example, if L is 20cm away and R is 15cm away, it's not significant. But if L is 20cm away and R is 10cm away, it is significant. This tells you it's time to readjust.
   c. speedAdjust: This is how much the motor will slow down when it's readjusting within walls.

----Notes----
2. Adjust color offset delay so robot is over the color. 
3. Make sure pins are correct. Color, line, and IR are currently just placeholders. 
4. I use TOP_MOTOR_SPEED for a lot of things. Don't change this variable but make another one if you think it's too fast.