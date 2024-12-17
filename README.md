# Table of Contents
- [Robot Modeling](#robot-modeling)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [License](#license)
- [Acknowledgements](#acknowledgements)
- [Contact](#contact)

------------------------------

# Robot Modeling
![GitHub](https://img.shields.io/github/license/adedhi/robot-modeling)

This is a 3D robot modeling project featuring a **Mobile Suit RX-78-2 Gundam**, created using the **OpenGL** library for the CPS511: Computer Graphics course at Toronto Metropolitan University. The project implements geometric transformations, hierarchical modeling, and simple animations for robot manipulation.

## Features
- **Multi-part Model**: The robot consists of multiple parts with 4 levels of hierarchy.
- **Controllable Joints**: Control key joints like the entire robot, head, arms, and legs using keyboard inputs.
- **Animations**:
  - **Walking Animation**: Triggered with the `w` key (stop with `W`), simulates a step forward by rotating hip and leg joints.
  - **Spinning Cannon Animation**: Activated with the `s` key (stop with `S`), continuously spins the cannon around its axis.
- **Hierarchical Transformations**: Uses OpenGL functions like `glTranslate()`, `glRotate()`, `glPushMatrix()`, and `glPopMatrix()`.

## Requirements
- Visual Studio 2022 (or compatible version)

## Installation
1. Download the 'Code' folder, which contains all the program files, visual studio files, and dependencies.

## Usage
1. Open Visual Studio
2. Click 'Open a Project or Solution'
3. Choose the 'Assignment 1.sln' file in the 'Code' folder
5. Once the project loads, click the Run button (green arrow) on the top of the screen (It says 'Local Windows Debugger' on Windows)
6. The project will open a debugging terminal (with instructions on how to use the program) and a separate window with the modelled robot

------------------------------

# Acknowledgements
- This program was created for a project in the CPS511: Computer Graphics course at Toronto Metropolitan University

# License
This project is licensed under the MIT License - learn more about it [here](LICENSE).

# Contact
If you have any questions, suggestions, or feedback, feel free to reach out to me at dadeshvir@gmail.com
