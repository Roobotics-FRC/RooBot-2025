package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GoTo extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final FieldCentric drive;
    private final double maxSpeed;

    // Initial PID constants
    private double translationalKP = 0.53;
    private double translationalKI = 0.001;
    private double translationalKD = 0.02;

    double poseX;
    double poseY;

    private double x;
    private double y;
    private double yaw;

    private final PIDController xPidController = new PIDController(translationalKP+0.02, translationalKI, translationalKD);
    private final PIDController yPidController = new PIDController(translationalKP, translationalKI, translationalKD);
    private final PIDController yawPidController = new PIDController(7, 0, 0.0);

    NetworkTable driveStateTable = NetworkTableInstance.getDefault().getTable("DriveState");
    StructSubscriber<Pose2d> poseSubscriber = driveStateTable.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d());

    public GoTo(CommandSwerveDrivetrain drivetrain, double maxSpeed, double x, double y, double yaw) {
        this.drivetrain = drivetrain;
        this.maxSpeed = maxSpeed;
        this.x = x;
        this.y = y;
        this.yaw = yaw;
        
        // Initialize the drive request with proper configuration
        this.drive = new SwerveRequest.FieldCentric();
        
        addRequirements(drivetrain);

        // Add PID constants to SmartDashboard
        SmartDashboard.putNumber("Translational KP", translationalKP);
        SmartDashboard.putNumber("Translational KI", translationalKI);
        SmartDashboard.putNumber("Translational KD", translationalKD);
    }

    @Override
    public void initialize() {
        config();
        setSetpoint(x, y, yaw);
    }

    @Override
    public void execute() {
        poseX = poseSubscriber.get().getX();
        poseY = poseSubscriber.get().getY();

        SmartDashboard.putNumber("Pose X", poseX-x);
        SmartDashboard.putNumber("Pose Y", poseY-y);
        //!For Testing Only - Remove in final code
        // Read PID constants from SmartDashboard
        translationalKP = SmartDashboard.getNumber("Translational KP", translationalKP);
        translationalKI = SmartDashboard.getNumber("Translational KI", translationalKI);
        translationalKD = SmartDashboard.getNumber("Translational KD", translationalKD);

        // Update PID controllers with new values
        xPidController.setP(translationalKP);
        xPidController.setI(translationalKI);
        xPidController.setD(translationalKD);

        yPidController.setP(translationalKP);
        yPidController.setI(translationalKI);
        yPidController.setD(translationalKD);
        //!For Testing Only - Remove in final code

        // Apply the fixed speeds
        drivetrain.setControl(
            drive.withVelocityX(xPidController.calculate(poseX) * maxSpeed)
                .withVelocityY(yPidController.calculate(poseY) * maxSpeed)
                .withRotationalRate(yawPidController.calculate(poseSubscriber.get().getRotation().getRadians()))
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Make sure to stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        
        translationalKP = SmartDashboard.getNumber("Translational KP", translationalKP);
        translationalKI = SmartDashboard.getNumber("Translational KI", translationalKI);
        translationalKD = SmartDashboard.getNumber("Translational KD", translationalKD);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("X At Setpoint", xPidController.atSetpoint());
        SmartDashboard.putBoolean("Y At Setpoint", yPidController.atSetpoint());
        SmartDashboard.putBoolean("Yaw At Setpoint", yawPidController.atSetpoint());

        return xPidController.atSetpoint() && yPidController.atSetpoint() && yawPidController.atSetpoint();
    }

    private void config() {
        xPidController.reset();
        yPidController.reset();
        yawPidController.reset();

        xPidController.setTolerance(0.05);
        yPidController.setTolerance(0.05);
        yawPidController.setTolerance(0.1);
        yawPidController.enableContinuousInput(-180, 180);
    }

    private void setSetpoint(double x, double y, double yaw) {
        xPidController.setSetpoint(x);
        yPidController.setSetpoint(y);
        yawPidController.setSetpoint(yaw);
    }
}