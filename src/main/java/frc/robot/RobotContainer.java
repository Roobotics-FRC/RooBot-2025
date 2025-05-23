// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import frc.robot.commands.Climb;
import frc.robot.commands.DeAlgee;
import frc.robot.commands.FeederGoTo;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveHoper;
import frc.robot.commands.OutTake;
import frc.robot.commands.RiefGoTo;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) //!! 20% For school controller || 10% For My Controller
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric() 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    private final Field2d field2d = new Field2d();
    private PathPlannerAuto currentAuto = null;
    private final List<String> activeTrajectories = new ArrayList<>();

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Joystick op_joystick = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LEDSubsystem ledSubsystem = new LEDSubsystem(0, 320);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final SuperstructureSubsystem m_SuperstructureSubsystem= new SuperstructureSubsystem(op_joystick);

    Command ElevatorDown = new MoveElevator(m_SuperstructureSubsystem, Positions.L0,false);
    Command HopperDown = new MoveHoper(m_SuperstructureSubsystem, -3, false);
    Command HopperUp = new MoveHoper(m_SuperstructureSubsystem, 1, true);

    Command L2DeAlgee = new MoveHoper(m_SuperstructureSubsystem, -3, false)
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L2A, false))
        .andThen(new DeAlgee(m_SuperstructureSubsystem, Positions.L2AE))
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0, false));
    
    Command L3DeAlgee = new MoveHoper(m_SuperstructureSubsystem, -3, false)
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L3A, false))
        .andThen(new DeAlgee(m_SuperstructureSubsystem, Positions.L3AE))
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0, false));
    
    Command L2Score = new MoveHoper(m_SuperstructureSubsystem, -3, true)
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L2, false))
        .andThen(new OutTake(m_SuperstructureSubsystem, false))
        .andThen(new WaitCommand(WaitTimes.scoreWait))
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0, false));
    
    Command L3Score = new MoveHoper(m_SuperstructureSubsystem, -3, true)
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L3, false))
        .andThen(new OutTake(m_SuperstructureSubsystem, false))
        .andThen(new WaitCommand(WaitTimes.scoreWait))
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0, false));
    
    Command L4Score = new MoveHoper(m_SuperstructureSubsystem, -3, true)
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L4, false))
        .andThen(new OutTake(m_SuperstructureSubsystem, false))
        .andThen(new WaitCommand(WaitTimes.scoreWait))
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0, false));

    Command L2Elevator = new MoveHoper(m_SuperstructureSubsystem, -3, true)
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L2, false));

    Command L3Elevator = new MoveHoper(m_SuperstructureSubsystem, -3, true)
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L3, false));
    
    Command L4Elevator = new MoveHoper(m_SuperstructureSubsystem, -3, true)
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L4+2, false));

    Command OutTake = new OutTake(m_SuperstructureSubsystem, false)
        .andThen(new WaitCommand(WaitTimes.scoreWait))
        .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0, false));
    
    Command OutTakeAuto = new OutTake(m_SuperstructureSubsystem, false)
        .andThen(new WaitCommand(WaitTimes.scoreWait));
    
    Command GoToRiefR =new RiefGoTo(drivetrain, MaxSpeed, Constants.Offsets.xRief, Constants.Offsets.yRiefR, ledSubsystem,joystick);

    Command GoToRiefL = new RiefGoTo(drivetrain, MaxSpeed, Constants.Offsets.xRief, Constants.Offsets.yRiefL, ledSubsystem,joystick);

    public RobotContainer() {
        //! Register the autonomous commands in here
        NamedCommands.registerCommand("Pre Raise", new MoveHoper(m_SuperstructureSubsystem, -3, true).andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L2, false)));
        NamedCommands.registerCommand("GoTo Rief L",  new RiefGoTo(drivetrain, MaxSpeed, Constants.Offsets.xRief-0.07, Constants.Offsets.yRiefL, ledSubsystem,joystick));
        NamedCommands.registerCommand("GoTo Rief R",  new RiefGoTo(drivetrain, MaxSpeed, Constants.Offsets.xRief-0.07, Constants.Offsets.yRiefR, ledSubsystem,joystick));
        NamedCommands.registerCommand("L3 Score", L3Score);
        NamedCommands.registerCommand("L2 Score", L2Score);
        NamedCommands.registerCommand("L4 Score", L4Score);
        NamedCommands.registerCommand("L2 DeAlgee", L2DeAlgee);
        NamedCommands.registerCommand("L3 DeAlgee", L3DeAlgee);
        NamedCommands.registerCommand("Hopper Up", HopperUp);
        NamedCommands.registerCommand("L4Elevator", L4Elevator);
        NamedCommands.registerCommand("OutTake", OutTake);
        NamedCommands.registerCommand("OutTake W/O", OutTakeAuto);
        NamedCommands.registerCommand("HoperUp W/O", new InstantCommand(() -> m_SuperstructureSubsystem.moveHoper(1)));
        NamedCommands.registerCommand("FeederGoTo", new FeederGoTo(drivetrain, MaxSpeed));
        NamedCommands.registerCommand("Elevator Down", new MoveElevator(m_SuperstructureSubsystem, Positions.L0, false));
        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        SmartDashboard.putData("Field", field2d);
        
        // Improved auto chooser listener
        autoChooser.onChange((command) -> {
            clearTrajectories();
            if (command instanceof PathPlannerAuto pathPlannerAuto) {
                currentAuto = pathPlannerAuto;
                try {
                    visualizeAuto(currentAuto);
                } catch (Exception e) {
                    System.err.println("Error visualizing auto: " + e.getMessage());
                }
            }
        });
    }
    
    private void clearTrajectories() {
        // Clear all existing trajectories
        activeTrajectories.forEach(name -> field2d.getObject(name).setPoses());
        activeTrajectories.clear();
    }
    
    private void visualizeAuto(PathPlannerAuto auto) throws ParseException {
        if (auto == null) return;
        
        try {
            var paths = PathPlannerAuto.getPathGroupFromAutoFile(auto.getName());
            if (paths == null || paths.isEmpty()) return;

            // Show starting pose
            var startingPose = paths.get(0).getStartingHolonomicPose();
            startingPose.ifPresent(pose -> field2d.getObject("startingPose").setPose(pose));
            
            // Show each path
            for (int i = 0; i < paths.size(); i++) {
                String trajectoryName = "trajectory" + (i == 0 ? "" : i);
                activeTrajectories.add(trajectoryName);
                
                // Set trajectory poses
                var trajectoryObject = field2d.getObject(trajectoryName);
                trajectoryObject.setPoses(paths.get(i).getPathPoses());
                
                // Note: Ally paths are not supported in the current PathPlanner version
            }
            
        } catch (IOException e) {
            System.err.println("Error loading auto file: " + e.getMessage());
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * (1-joystick.getRightTriggerAxis()/2)) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * (1-joystick.getRightTriggerAxis()/2)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * (1-joystick.getRightTriggerAxis()/2)) // Drive counterclockwise with negative X (left)
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

        //! Reset North
        // // reset the field-centric heading on left bumper press
        new JoystickButton(op_joystick, 2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.x().whileTrue(GoToRiefL);
        joystick.b().whileTrue(GoToRiefR);
        joystick.a().whileTrue(new RiefGoTo(drivetrain, MaxSpeed, -1, 0, ledSubsystem,joystick));
        joystick.rightBumper().whileTrue(new FeederGoTo(drivetrain, MaxSpeed));
        joystick.leftBumper().whileTrue(new FeederGoTo(drivetrain, MaxSpeed));

        //Left Side Front
        new JoystickButton(op_joystick, 5).onTrue(L2Elevator); //L2
        new JoystickButton(op_joystick, 5).onFalse(OutTake);

        new JoystickButton(op_joystick, 6).onTrue(L3Elevator); //L3
        new JoystickButton(op_joystick, 6).onFalse(OutTake);

        new JoystickButton(op_joystick, 7).onTrue(L3Elevator); //L4
        new JoystickButton(op_joystick, 7).onFalse(new MoveElevator(m_SuperstructureSubsystem, Positions.L4, false)
                                                                .andThen(new OutTake(m_SuperstructureSubsystem, false))
                                                                .andThen(new WaitCommand(WaitTimes.scoreWait))
                                                                .andThen(new MoveElevator(m_SuperstructureSubsystem, Positions.L0, false)));

        //Left Side Back
        new JoystickButton(op_joystick, 8).whileTrue(L3DeAlgee);
        new JoystickButton(op_joystick, 9).whileTrue(L2DeAlgee);
        new JoystickButton(op_joystick, 10).onTrue(ElevatorDown);

        //Right Side Front
        new JoystickButton(op_joystick, 11).whileTrue(new MoveHoper(m_SuperstructureSubsystem, -3, false).alongWith(new Climb(m_SuperstructureSubsystem, -8)));
        new JoystickButton(op_joystick, 12).whileTrue(new MoveHoper(m_SuperstructureSubsystem, -3, false).alongWith(new Climb(m_SuperstructureSubsystem, -3)));
        new JoystickButton(op_joystick, 13).whileTrue(new MoveHoper(m_SuperstructureSubsystem, -3, false).alongWith(new Climb(m_SuperstructureSubsystem, 8)));

        //Right Side Back
        new JoystickButton(op_joystick, 14).onTrue(HopperDown);
        new JoystickButton(op_joystick, 15).onTrue(HopperUp);

        // //Left Side Front
        // new JoystickButton(op_joystick, 5).onTrue(L2Score);
        // new JoystickButton(op_joystick, 6).onTrue(L3Score);
        // new JoystickButton(op_joystick, 7).onTrue(L4Score);
        // //Left Side Back
        // new JoystickButton(op_joystick, 8).whileTrue(L3DeAlgee);
        // new JoystickButton(op_joystick, 9).whileTrue(L2DeAlgee);
        // new JoystickButton(op_joystick, 10).onTrue(ElevatorDown);

        // //Right Side Front
        // new JoystickButton(op_joystick, 11).whileTrue(new MoveHoper(m_SuperstructureSubsystem, -3).andThen(new Climb(m_SuperstructureSubsystem, -5)));
        // new JoystickButton(op_joystick, 12).whileTrue(new MoveHoper(m_SuperstructureSubsystem, -3).andThen(new Climb(m_SuperstructureSubsystem, -3)));
        // new JoystickButton(op_joystick, 13).whileTrue(new MoveHoper(m_SuperstructureSubsystem, -3).andThen(new Climb(m_SuperstructureSubsystem, 5)));

        // //Right Side Back
        // new JoystickButton(op_joystick, 14).onTrue(HopperDown);
        // new JoystickButton(op_joystick, 15).onTrue(HopperUp);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    public void autonomousInit() {
        ledSubsystem.setAutonomousMode(); 
    }
    
    public void teleopInit() {
        ledSubsystem.setTeleopMode();
    }
    
    public void disabledInit() {
        ledSubsystem.setDisabledMode();
    }
    
    public void robotInit(){
        ledSubsystem.setDisabledMode();
    }
    
    public void periodic() {
        // Update robot pose and orientation
        var robotPose = drivetrain.getState().Pose;
        field2d.setRobotPose(robotPose);
        
        // Update starting pose visibility based on game state
        var startingPoseObject = field2d.getObject("startingPose");
        if (!DriverStation.isDisabled()) {
            // Hide starting pose during match
            startingPoseObject.setPoses();
        }
    }
}
