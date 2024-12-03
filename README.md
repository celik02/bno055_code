### C++ Code to Configure and Read Data From BNO055 IMU
Although Adafruit supplies module for IMU configuration in Python, I could not find something to the same tasks in C++ so I wrote it myself. 
Basically this code configures IMU to desired state by writing to registers and reads data from IMU. Communication is done with I2C library. 
