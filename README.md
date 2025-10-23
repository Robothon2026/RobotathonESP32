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

----Notes----
1. Make sure you can switch to manual control whenever you want. This wil put you into an infinite loop otherwise in case of a bug.
2. Adjust color offset delay o robot is over the color.