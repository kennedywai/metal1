# Overview
* Base Test Procedure without ROS
* Base Test Procedure with ROS

## Sonar wiring on mega base
  * Replaced with 8 sonars only.
![](https://github.com/piliwilliam0306/metal1/blob/andbot2/mcu_control/base_control/sonar_new.jpg)

# Base Test Proceduce without ROS
## Modify Arduino PWM Frequency in Arduino IDE
  * Overwrite wiring.c in: 
     ~/arduino-1.6.5/hardware/arduino/avr/cores/arduino
  * Edit line 31 in "wiring.c" (For Vnh5019 board only)
     #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(8 * 256))

## Uploading mega code
  * Copy "ros_lib" folder into "Arduino/libraries" folder.
  * "mega_base_v1" is for base mega board.
  * "#define ANDBOT 1" when uploading Andbot.
  * "#define RUGBY 2" when uploading Rugby or Angel.
  * Select Mega2560 when uploading.

## Uploading motor driver code
  * "vnh5019_base_test.ino" is to test if motor can achieve desire speed. (Speed control test)
  * "#define WHEEL_TYPE RIGHT_WHEEL" when uploading for right wheel. 
  * "#define WHEEL_TYPE LEFT_WHEEL" when uploading for left wheel.
  * "#define ANDBOT 3" when uploading Andbot.
  * "#define RUGBY 4" when uploading Rugby or Angel.

## Test speed control
  * Enter 4 charactors target speed in Arduino Serial Console/Window and select 56700Hz for baudrate.
     Any value from -10.9 to +10.9 for Andbot, Any value from -31 to +31 for Rugby or Angel
  * If "actual speed" is close to "target speed," wiring and control board are working; if not, check wiring or swap encoder pins.
  * "vnh5019_base.ino" is for the motor controller board for base. (Upload this if vnh5019 control board pass speed test.)
  * Select Arduino Pro Mini when uploading.


# Base Test Proceduce with ROS
## Hardware
  * Connect Vnh5019 board to mega board
  * make sure to wire "Serial2" to LEFT Vnh5019 driver board and "Serial1" to RIGHT.
  * Connect mega board to Odroid board via USB
  * Connect HDMI monitor, mouse and keyboard to Odroid board
  * Power on Odroid board, mega board and Vnh5019 board
  * Prepare an Android phone, and install teleop app

## Check Odroid IP adress (In Odroid board)
     $ ifconfig  

## Compiling packages (in ssh terminal)
     $ ssh odroid@[Odroid IP address]
     $ cd ~/catkin_ws/src
     $ git clone https://github.com/piliwilliam0306/metal1.git
     $ cd ~/catkin_ws
     $ catkin_make
     $ rospack profile
     
## Test Rosserial (in ssh terminal)
     $ roscore
     $ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=1000000
     $ rostopic pub /cmd_wheel_angularVel andbot/WheelCmd "speed1: 2.0 speed2: 2.0 driverstate: true" -r 100
     $ rostopic echo feedback_wheel_angularVel
      * if "feedback_wheel_angularVel" topic output is close to "cmd_wheel_angularVel," rosserial and motor control board is working. (speed1 is for "left wheel," speed2 is "right wheel.")
     $ rostopic pub /cmd_wheel_angularVel andbot/WheelCmd "speed1: 0.0 speed2: 0.0 driverstate: true"
     $ Ctrl + C to stop
     $ rostopic echo /sonar (check if sonars work)

## Make robot move via Teleop app
  * Now, put the robot on the ground
  * In ssh terminal in Odroid board, launch startup script
     $ roslaunch andbot andbot2.launch (launch andbot1.launch if you are using andbot #1)
  * Open Teleop app in Android phone
    * Enter "http://[Odroid IP address]:11311 as ROS_MASTER_URI. 
  * Now you should see a map in the Teleop app
  * Use the teleop app to control the robot movement/rotation for a few meters, and check if the map updating is correct.
 
     
