# ROS Wrapper for Senkylaser Skpro-30

## Description
This is a package for using Senkylaser [Skpro30](http://www.shsenky.com/index.php?c=show&id=49) with ROS. The package is also suitable for Skpro60 and Skpro100.

## Dependiencies
- ROS melodic/kinetic
- [libmodbus](https://github.com/stephane/libmodbus)
- [FTDI TN_101 Linux](https://www.ftdichip.com/Support/Documents/TechnicalNotes/TN_101_Customising_FTDI_VID_PID_In_Linux(FT_000081).pdf) (RS485 to USB)

## Instruction
1. Install the RS485 to USB linux driver, most of the chips use FTDI, so you can refer to [FTDI TN_101](https://www.ftdichip.com/Support/Documents/TechnicalNotes/TN_101_Customising_FTDI_VID_PID_In_Linux(FT_000081).pdf) 
2. Install the ROS. [Ubuntu install of ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
3. installl [libmodbus](https://github.com/stephane/libmodbus)
    
     Follow the [instruction](https://github.com/stephane/libmodbus) 

4. Create a catkin workspace Unbuntu

     ```batch
     mkdir -p ~/catkin_ws/src
     cd ~/catkin_ws/src
     ```
5. Clone this respository into 'catkin_ws/src'

     ```batch
     git clone https://github.com/nkuhzx/senkylaser-skpro-ros.git
     cd ~/catkin_ws
     catkin_make
     ```
6. Connect your senkylaser and roslaunch (RS485->USB->Laptop)
    
     ```batch
     source devel/setup.bash
     roslaunch skpro30_laser lasernode.launch 
     ```

7. Then the topic is published on '/laser_node/sklaser' as sensor_msg::LaserScan

     ```batch
     source devel/setup.bash
     roslaunch skpro30_laser lasernode.launch 
     ```

## F&Q
1. Q: How to connect Senkylaser to my computer?
    
   A: You need an adapter to convert RS485 to USB serial port.

2. Q: Cannot open /dev/ttyusb0 no such file or directory
   
   A: First check the properties of ttyusb0

    ```batch
    ll /dev/ttyUSB0
    ``` 
      Then

    ```batch
    sudo chmod 666 /dev/ttyUSB0
    ```
    It is valid only once, and the command needs to be executed again after unplugging the cable.

    or
    ```batch
    sudo usermod -aG dialout USERNAME
    ```    
    USERNAME is your ubuntu username, this method is permanently valid.

3. How to check whether the device can communicate with serial port

    Use [cutecom](http://cutecom.sourceforge.net/) to open '/dev/ttyUSB0', and then power on the device. If you can see the baud rate, it means the connection is successful.

4. What are the contents of the published message?

    The measured distance is in the 'ranges' of the LaserScan message.
    
    We use 'intensities' to indicate error status.

    

## Author
- Zhengxi Hu

Please raise an issue or send email to hzx@mail.nankai.edu.cn if there are any issues running the code.