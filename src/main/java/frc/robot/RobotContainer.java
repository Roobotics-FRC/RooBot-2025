// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Positions;
import frc.robot.Constants.WaitTimes;
import frc.robot.commands.DeAlgee;
import frc.robot.commands.GoTo;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveHoper;
import frc.robot.commands.OutTake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperstructureSubsystem;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Joystick op_joystick = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final SuperstructureSubsystem m_SuperstructureSubsystem= new SuperstructureSubsystem(op_joystick);

    Command ElevatorDown = new MoveElevator(m_SuperstructureSubsystem, Positions.L0,false);
    Command HopperDown = new MoveHoper(m_SuperstructureSubsystem, -3);
    Command HopperUp = new MoveHoper(m_SuperstructureSubsystem, 0.5);
    Command ClimbDown = new InstantCommand(() -> m_SuperstructureSubsystem.climbDown());
    Command ClimbUp = new InstantCommand(() -> m_SuperstructureSubsystem.climbUp(120));

    Command L2DeAlgee = new MoveElevator(m_SuperstructureSubsystem, Positions.L2A, false).andThen(new DeAlgee(m_SuperstructureSubsystem, Positions.L2AE)).andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0,false));
    Command L3DeAlgee = new MoveElevator(m_SuperstructureSubsystem, Positions.L3A, false).andThen(new DeAlgee(m_SuperstructureSubsystem, Positions.L3AE)).andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0,false));
    Command L2Score = new MoveElevator(m_SuperstructureSubsystem, Positions.L3, false).andThen(new OutTake(m_SuperstructureSubsystem,false)).andThen(new WaitCommand(WaitTimes.scoreWait)).andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0,false));
    Command L3Score = new MoveElevator(m_SuperstructureSubsystem, Positions.L3, false).andThen(new OutTake(m_SuperstructureSubsystem,false)).andThen(new WaitCommand(WaitTimes.scoreWait)).andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0,false));
    Command L4Score = new MoveElevator(m_SuperstructureSubsystem, Positions.L4, false).andThen(new OutTake(m_SuperstructureSubsystem,false)).andThen(new WaitCommand(WaitTimes.scoreWait)).andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L4E,true).andThen(new WaitCommand(0.2).andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0,false))));


    public RobotContainer() {
        //! Register the autonomous commands in here
        NamedCommands.registerCommand("Go To Rief 1", new GoTo(drivetrain,MaxSpeed,5.269,3.027,0, m_SuperstructureSubsystem));
        NamedCommands.registerCommand("Go To Rief 2", new GoTo(drivetrain,MaxSpeed,4.006,2.853,0, m_SuperstructureSubsystem));
        NamedCommands.registerCommand("GoTo", new GoTo(drivetrain,MaxSpeed,3.15,3.95,0, m_SuperstructureSubsystem));
        NamedCommands.registerCommand("L3 Score", L3Score);
        NamedCommands.registerCommand("L2 Score", L2Score);
        NamedCommands.registerCommand("L2 Score", L4Score);
        NamedCommands.registerCommand("L2 DeAlgee", L2DeAlgee);
        NamedCommands.registerCommand("L3 DeAlgee", L3DeAlgee);
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.75) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.75) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 1) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.x().whileTrue(new GoTo(drivetrain,MaxSpeed,3.15,3.95,0, m_SuperstructureSubsystem));

        new JoystickButton(op_joystick, 5).onTrue(L2Score);
        new JoystickButton(op_joystick, 6).onTrue(L3Score);
        new JoystickButton(op_joystick, 7).onTrue(L4Score);
        new JoystickButton(op_joystick, 10).onTrue(ElevatorDown);
        new JoystickButton(op_joystick, 11).whileTrue(new OutTake(m_SuperstructureSubsystem,true));
        new JoystickButton(op_joystick, 12).whileTrue(L2DeAlgee);
        new JoystickButton(op_joystick, 13).whileTrue(L3DeAlgee);
        new JoystickButton(op_joystick, 14).onTrue(HopperDown);
        new JoystickButton(op_joystick, 15).onTrue(HopperUp);
        new JoystickButton(op_joystick, 16).onTrue(ClimbDown);
        //new JoystickButton(op_joystick, 17).onTrue(ClimbUp);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
