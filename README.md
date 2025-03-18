# RooBot-2025 🤖

Welcome to the RooBot-2025 repository! This is the code for Team Roobotics' 2025 FRC competition robot. Our robot is built using Java and implements advanced features for autonomous operation, precise control, and competitive gameplay.

## 🎮 Features

### Drivetrain
- Swerve drive system with field-centric control
- Precision movement and rotation using PID controllers
- Support for both autonomous and teleoperated control
- Field-relative positioning using AprilTag vision targeting

### Autonomous Capabilities 🤖
- PathPlanner integration for complex autonomous routines
- Dynamic path visualization on Field2d
- Multiple autonomous modes selectable via SmartDashboard
- AprilTag-based autonomous alignment and positioning
- Support for both alliance colors (Red/Blue)

### Superstructure 🏗️
- Elevator system with dual TalonFX motors
  - Precise position control
  - Configurable PID settings
  - Multiple preset positions (L2, L3, L4)
- Climbing mechanism with TalonFX motor
  - Position-based control
  - Current limiting for safety
- Intake system with SparkMax controllers
  - Game piece manipulation
  - De-algee functionality
  - Hopper control system

### LED System 💡
The robot features a sophisticated LED feedback system with multiple states:
- **Autonomous Mode**: Breathing gold pattern
- **Teleop Mode**: Alliance color breathing pattern
- **Alignment Mode**: Blinking green
- **Disabled Mode**: Solid alliance color
- **Game Piece Status**:
  - Intaking: Blinking orange
  - Has Game Piece: Solid green
  - Ready to Score: Blinking purple

### Vision and Targeting 👁️
- Limelight integration for AprilTag detection
- Support for 2025 Reefscape field AprilTags (IDs 6-11 and 17-22)
- Automated alignment with scoring locations
- Real-time pose estimation and field positioning

## 🛠️ Commands and Control

### Driver Controls
- Field-centric driving with joystick input
- Maximum speed and angular rate control
- Quick-turn and precision movement modes
- Multiple scoring position presets

### Operator Controls
#### Left Side Front
- Button 5: L2 Elevator + Outtake
- Button 6: L3 Elevator + Outtake
- Button 7: L4 Elevator + Outtake

#### Left Side Back
- Button 8: L3 De-algee
- Button 9: L2 De-algee
- Button 10: Elevator Down

#### Right Side Controls
- Buttons 11-13: Climbing controls
- Button 14: Hopper Down
- Button 15: Hopper Up

## 📝 License

This project is licensed under WPILib's BSD license. See the LICENSE file for details.

## 🏆 Team

Made with ❤️ by Team Roobotics - FRC Team [4373]
