# Code Readme

This code repository contains the working code for an ESP32-controlled car that moves according to on-board sensors and voice commands. It consists of 4 main folders, buggy, dictaphone, pi_stream, and server.

The pi_stream folder contains the server that hosts the RPi cam stream that is run on an onboard RPi.

The dictaphone folder contains a frontend React app that uses the react-speech-recognition npm module to transcribe speech coming from the laptop microphone to text. It then uses a POST method to transfer special keywords to the backend NodeJS server.

This backend NodeJS server lies in the server folder, and takes in the keywords from the frontend through a GET method, and displays it on a simple HTML page. It then transmits this data over to the onboard ESP32 through a UDP socket connection.

Finally, the buggy folder contains the code for controlling the car sensors, ESC, and steering servo. It first starts the calibration sequence for the car, then executes a few main tasks. There is a timer task that is used by the PID calculations from the rotary encoder and lidar data, a udp task to obtain commands, and a command handler task that modifies the global speed and heading variables. This allows for the robot to move continuously, all based on its surroundings and obstacles.

