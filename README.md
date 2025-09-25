# 2DOF Plotter Control System

Welcome to the **2DOF_Plotter_Control_SYS** repository! This project is a modular, C-based control system for a two-degree-of-freedom (2DOF) plotter, developed as part of advanced robotics coursework at King Mongkut’s University of Technology Thonburi (KMUTT), Institute of Field Robotics (FIBO).

---

## Project Overview

This repository contains firmware and control algorithms for a 2DOF robotic plotter. The system is designed for educational and research purposes, focusing on trajectory planning, control strategies, and real-time performance evaluation.

- **Languages:** C (99.4%), Assembly (0.6%)
- **Target Audience:** Robotics students, educators, and researchers interested in embedded control systems and kinematic applications.

---

## Detailed Directory and File Structure

- **Core/**
  - Contains core control logic and main routines, including velocity control and other essential functions.
- **kalman/**
  - Implements Kalman filter algorithms for sensor fusion and state estimation.
- **log/**
  - Logging utilities and recorded experiment data, including logs for different control experiments.
- **src/**
  - Source code for hardware interfacing, such as ModBusRTU communication protocols.
- **Daikalman_main.c**
  - Main program for Kalman filter-based control experiments.
- **Plotter.ioc**
  - STM32CubeMX project configuration file for hardware setup.
- **Schematic.jpg**
  - Hardware schematic of the 2DOF plotter system.
- **Two_joint_main.c**
  - Main program for controlling both joints of the plotter simultaneously.
- **full_revolute_main.c**
  - Main program for full revolute joint control.
- **full_revolute_main_trajec.c**
  - Trajectory generation and control for full revolute configuration.
- **full_revolute_main_trajecV2.c**
  - Alternative version for trajectory generation and control.
- **main_FIBO.c**
  - Main control program for FIBO-specific experiments.
- **main_FIBOG08.c**
  - Main program for G08 group experiments.
- **main_G08.c**
  - G08 group main control program.
- **main_Home.c**
  - Homing routine for initializing the plotter’s position.
- **main_Inverse_traject.c**
  - Main program for inverse kinematics trajectory generation.
- **main_cascade_control.c**
  - Implements cascade control strategy for the 2DOF plotter.
- **main_circle.c**
  - Main program for generating and following circular trajectories.
- **main_finalTreject.c**
  - Final version for trajectory rejection testing and validation.
- **main_full_com.c**
  - Main program for comprehensive test scenarios and combined trajectory experiments.
- **main_full_version_forsontrol.c**
  - Full version main program for advanced control strategies.
- **main_fullkalman.c**
  - Main program for full-state Kalman filter-based control.
- **main_trajectoryandbohjoint.c**
  - Main program for trajectory tracking with both joints.
- **main_velocheck.c**
  - Velocity checking and monitoring main program.
- **revolute_prismatic_main.c**
  - Main program for combined revolute and prismatic joint configuration.
- **tuning_full_revolute_main_trajecV2_pris.c**
  - Tuning and testing for full revolute and prismatic joint trajectories.

---

## Key Features

- Modular control frameworks: cascade, full-state, and Kalman-based controllers
- Multiple trajectory generation algorithms (circular, linear, custom)
- Real-time velocity and position monitoring
- Data logging for analysis and debugging
- Expandable for additional sensors or actuators

---

## Getting Started

1. **Clone the repository:**  
   `git clone https://github.com/Lworakan/2DOF_Plotter_Control_SYS.git`
2. **Open in STM32CubeIDE or a compatible IDE.**
3. **Review the schematic and wiring (see `Schematic.jpg`).**
4. **Build and flash the firmware to your STM32-based controller.**

---

## Example Use Cases

- Implement and compare different control strategies for a 2DOF robotic arm  
- Study the effect of state estimation using Kalman filters  
- Experiment with trajectory generation and real-time plotting  
- Collect and analyze system logs to evaluate performance  

---

## Contributions

Contributions are welcome! Please open an issue or submit a pull request if you have improvements, bug fixes, or new features.

---

## Acknowledgements

Developed by Worakan Lasudee (@Lworakan) as part of FIBO-KMUTT’s advanced robotics curriculum. Special thanks to classmates, professors, and the FIBO community for their support and inspiration.

---

## License

This project is open-source for educational and research purposes. Please cite or acknowledge if used in academic work.

---

## Contact

For questions or collaboration, contact:  
- **Worakan Lasudee**  
- Email: worakan.lasudee@gmail.com / worakan.lasu@kmutt.ac.th  
- LinkedIn: [Worakan Lasudee](https://www.linkedin.com/in/worakan-lasudee/)
