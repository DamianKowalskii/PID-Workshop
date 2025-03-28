# PID Workshop

A Qt-based educational application for visualizing and experimenting with PID controller tuning. The program uses a car model with cruise control as an example system to demonstrate how PID parameters affect controlled system behavior.

![PID Workshop Screenshot](pid_workshop.png)

## Features

- Real-time visualization of:
  - System output (vehicle velocity)
  - Setpoint tracking
  - Control signal (PID controller output)
- Interactive PID parameter adjustment:
  - Proportional gain (Kp)
  - Integral gain (Ki) 
  - Derivative gain (Kd)
- Anti-windup control option
- Automatic PID tuning using compact genetic algorithm
- Configurable simulation time step

## Implementation Details

- Built with Qt and C++
- Uses QCustomPlot for real-time data visualization
- Implements car dynamics model as the controlled plant
- Features an autotuning algorithm based on metaheuristic optimization

## Building

The project requires:
- Qt framework
- C++ compiler with C++11 support

## Usage

1. Launch the application
2. Adjust PID parameters using the spinboxes
3. Set target velocity (setpoint)
4. Click "Start" to begin simulation
5. Observe system response in real-time plots
6. Use "Stop" and "Restart" to control simulation
7. Try "Start autotuning" for automated parameter optimization

## License

This project is released under the MIT License. See LICENSE file for details.