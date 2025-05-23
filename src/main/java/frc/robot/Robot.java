// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final Field2d field2d = new Field2d();

  public Robot() {
        m_robotContainer = new RobotContainer();
        // Get the Field2d instance from SmartDashboard
        
        SmartDashboard.putData("Field", field2d);
        m_robotContainer.robotInit();
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.periodic();
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    m_robotContainer.disabledInit();
  }


  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.autonomousInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    field2d.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.teleopInit();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
