# Ball Beam Balancer

A ball-and-beam balancing rig built to test PID control with disturbance rejection: a servo tilts the beam to keep the ball at a setpoint distance, measured by a time-of-flight sensor.

## Contents

- **[Fusion files/](Fusion%20files)** – Fusion 360 CAD exports (STL) for the motor arm and time-of-flight sensor fixture. Carbon fiber rods were used to connect the "beam".
- **[Time of flight sensor PID/vl53l0x.ino](Time%20of%20flight%20sensor%20PID/vl53l0x.ino)** – Arduino sketch implementing the PID control loop: reads distance from a VL53L0X time-of-flight sensor, filters it, and drives a servo via microsecond pulse width to balance the ball.
- **[Video](Video)** – Demo video of the balancer in operation.
