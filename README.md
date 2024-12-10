Automated Aiming Device Simulation


Project Overview：

This project demonstrates a simulation-based automated aiming system. The system uses PyBullet for robotic arm simulation, integrates a virtual camera for target detection, and employs a control algorithm to track and aim at a moving target.


The key features include:

A robotic arm simulation using the KUKA iiwa URDF model.

Real-time target tracking with smooth movements.

Simulation of environmental constraints (e.g., obstacles).


Dependencies：

The project relies on the following Python libraries:

PyBullet (version 3.25): For physics simulation.

Numpy: For numerical computations

Features：

Robotic Arm Simulation:

Uses PyBullet to simulate the KUKA iiwa robotic arm.

Target Tracking:

Detects and tracks a moving sphere in a dynamic environment.

Obstacle Avoidance:

Ensures the robot arm avoids aiming at obstructed targets using ray-casting.

Virtual Camera Integration:

Simulates a camera to detect and track the target.


References：

PyBullet Documentation: https://pybullet.org/

Official Bullet Physics GitHub: https://github.com/bulletphysics/bullet3
