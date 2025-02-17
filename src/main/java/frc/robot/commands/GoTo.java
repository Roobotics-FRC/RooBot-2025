package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GoTo extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final FieldCentric drive;
    private final double maxSpeed;

    private final PIDController xPidController = new PIDController(1, 0.0, 0.0);
    private final PIDController yPidController = new PIDController(1, 0.0, 0.0);
    private final PIDController yawPidController = new PIDController(10, 0.0, 0.0);

    NetworkTable driveStateTable = NetworkTableInstance.getDefault().getTable("DriveState");
    StructSubscriber<Pose2d> poseSubscriber = driveStateTable.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d());

    public GoTo(CommandSwerveDrivetrain drivetrain, double maxSpeed) {
        this.drivetrain = drivetrain;
        this.maxSpeed = maxSpeed;
        
        // Initialize the drive request with proper configuration
        this.drive = new SwerveRequest.FieldCentric();
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        config();
        setSetpoint(3, 4, 0);
    }

    @Override
    public void execute() {
        System.out.println(poseSubscriber.get());
        // Apply the fixed speeds
        drivetrain.setControl(
            drive.withVelocityX(xPidController.calculate(poseSubscriber.get().getX()) * maxSpeed)
                .withVelocityY(yPidController.calculate(poseSubscriber.get().getY()) * maxSpeed)
                .withRotationalRate(yawPidController.calculate(poseSubscriber.get().getRotation().getRadians()))
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Make sure to stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    @Override
    public boolean isFinished() {
        return xPidController.atSetpoint() && yPidController.atSetpoint() && yawPidController.atSetpoint();
    }

    private void config() {
        xPidController.reset();
        yPidController.reset();
        yawPidController.reset();

        xPidController.setTolerance(0.01);
        yPidController.setTolerance(0.01);
        yawPidController.setTolerance(0.01);
        yawPidController.enableContinuousInput(-180, 180);
    }

    private void setSetpoint(double x, double y, double yaw) {
        xPidController.setSetpoint(x);
        yPidController.setSetpoint(y);
        yawPidController.setSetpoint(yaw);
    }
}