Ever wondered how a Segway manages to stay upright? This project was my attempt at building a miniature version using an ESP32, MPU6050, and a pair of DC geared motors.

The robot constantly measures its tilt using the MPU6050 and uses a PID controller to decide how the motors should react. If it starts falling forward, the wheels move forward to catch it. If it falls backward, the wheels move backward. Sounds simple, but getting it to balance was a lot harder than it looked.

Most of the work went into tuning the PID controller, experimenting with sensor filtering, fixing motor directions, adjusting weight distribution, and figuring out why the robot would randomly decide that gravity deserved to win. Through countless iterations, the robot was eventually able to detect tilt and actively attempt to balance itself in real time.

Hardware Used
ESP32
MPU6050
L298N Motor Driver
2 DC Geared Motors
18650 Battery Pack
Foam Board Chassis
What I Learned
PID Control
Sensor Fusion using Accelerometer and Gyroscope Data
Embedded Programming with ESP32
Motor Control
Real-Time Robotics
Debugging Hardware and Control Systems

This project taught me that building a robot is easy. Getting it to behave the way you want is the real challenge.
