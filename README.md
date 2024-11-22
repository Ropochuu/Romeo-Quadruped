# Romeo-Quadruped
This is a repository for Romeo's quadruped 

#Description  
 This Quadruped is a robotic system that features a (four-legged) robot controlled by a series of 8 servos. Each leg is composed of two servos: one for flexion (ankle) and one for side movement (hip). The servos are controlled via a Raspberry Pi Pico, and a Python-based PC application allows the user to update the leg positions in real time through serial communication. The system supports the saving and loading of robot states in JSON format, allowing for easy configuration and control of leg positions.


# Features
Real-time Control: Control the robot’s legs via sliders on the PC interface.
Servo Control: Adjust the position of each servo to move the quadruped robot’s legs.
JSON State Management: Save and load robot positions using JSON files for easy state management.
Raspberry Pi Pico Integration: Uses a Raspberry Pi Pico for servo control with MicroPython.
Customizable Movement: Change the position of each leg individually for custom movements.

# Instructions
1. Setup Hardware
   
Connect the servos to the Raspberry Pi Pico as per the pin mapping defined in the code.
Ensure that the servos are powered and connected to the correct GPIO pins.

2. Install libraries
   
Make sure you have the necessary libraries installed on the Raspberry Pi Pico and your PC.
On the Raspberry Pi Pico, install MicroPython and any required libraries for PWM and serial communication.

3. Run the Pico Application
   
Open the Pico application on Thonny to run the main loop to the pico.
Then close Thonny while the main loop is running
The Pico will listen in the background for JSON commands from the pc application.


4. Run the PC Application
   
Open the Python-based application on your PC to control the servos.
The application will send the servo control commands over a serial connection to the Raspberry Pi Pico.
Adjust the sliders to move the servos and position the robot’s legs.

5.Load and Save State

You can save the current positions of the servos as a JSON file and load them back into the system.
The Application has 2 states already pre built into the program 
-Standing
-Waving

6.Test the Robot

Once the application and hardware are set up, test the robot’s movements by adjusting the sliders or loading saved states.

# Video Demo

see main

#External Libraries

MicroPython: The Raspberry Pi Pico runs MicroPython to control the servos via PWM.
PWM Library: For controlling the servo positions on the Raspberry Pi Pico.
JSON: Used to serialize and deserialize the robot’s state on both the Raspberry Pi Pico and PC.
Time: Provides the time.sleep function to manage delay and control servo updates.
