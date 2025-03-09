package frc.robot.commands;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;

public class AutoGoTo extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final FieldCentric drive;
    private final LEDSubsystem ledSubsystem;
    private final double maxSpeed;
    private final double relativeX;
    private final double relativeY; 
    
    private double targetX;
    private double targetY;
    private double targetYaw;

    double rotationalKP;

    private AprilTagFieldLayout fieldLayout;
    private int closestTagId = -1;

    // Valid AprilTag IDs (6-11 and 17-22)
    private static final Set<Integer> VALID_TAG_IDS = Set.of(
        6, 7, 8, 9, 10, 11,
        17, 18, 19, 20, 21, 22
    );
    
    private final PIDController xPidController = new PIDController(Constants.PID.translationalKP, Constants.PID.translationalKI, Constants.PID.translationalKD);
    private final PIDController yPidController = new PIDController(Constants.PID.translationalKP, Constants.PID.translationalKI, Constants.PID.translationalKD);
    private final PIDController yawPidController = new PIDController(0.2, Constants.PID.rotationalKI, Constants.PID.rotationalKD);

    /**
     * Creates a new AutoGoTo command that goes to the closest valid AprilTag
     * @param drivetrain The swerve drivetrain subsystem
     * @param maxSpeed Maximum speed for movement
     * @param relativeX Distance to maintain in X direction relative to the tag in meters
     * @param relativeY Distance to maintain in Y direction relative to the tag in meters
     * @param superstructureSubsystem The superstructure subsystem for LEDs
     */
    public AutoGoTo(CommandSwerveDrivetrain drivetrain, double maxSpeed, double relativeX, double relativeY, LEDSubsystem ledSubsystem) {
        this.drivetrain = drivetrain;
        this.maxSpeed = maxSpeed;
        this.relativeX = relativeX;
        this.relativeY = relativeY;
        this.ledSubsystem = ledSubsystem;
        this.drive = new SwerveRequest.FieldCentric();
        
        addRequirements(drivetrain);
        
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTag field layout", e.getStackTrace());
        }
        
        config();
        SmartDashboard.putNumber("Rotations", 0.2);
    }

    @Override
    public void initialize() {
        resetPID();
        findClosestAprilTag();
        if (closestTagId != -1) {
            updateTargetPosition();
        }
        rotationalKP = SmartDashboard.getNumber("Rotations", rotationalKP);
    }

    @Override
    public void execute() {
        yawPidController.setP(rotationalKP);
        if (closestTagId == -1) {
            findClosestAprilTag();
            if (closestTagId != -1) {
                updateTargetPosition();
            }
            return;
        }

        Pose2d currentPose = drivetrain.getState().Pose;

        drivetrain.setControl(
            drive.withVelocityX(xPidController.calculate(currentPose.getX()) * maxSpeed)
                .withVelocityY(yPidController.calculate(currentPose.getY()) * maxSpeed)
                .withRotationalRate(yawPidController.calculate(currentPose.getRotation().getDegrees()))
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        // Add this to ensure we return to alliance color even if interrupted
        ledSubsystem.setTeleopMode();
    }

    @Override
    public boolean isFinished() {
        return xPidController.atSetpoint() && yPidController.atSetpoint() && yawPidController.atSetpoint();
    }

    private void resetPID() {
        xPidController.reset();
        yPidController.reset();
        yawPidController.reset();
    }

    private void config() {
        xPidController.setIZone(Constants.PID.thanslationalIZone);
        yPidController.setIZone(Constants.PID.thanslationalIZone);

        xPidController.setTolerance(Constants.PID.translationalTolerance);
        yPidController.setTolerance(Constants.PID.translationalTolerance);
        yawPidController.setTolerance(Constants.PID.rotationalTolerance);
        yawPidController.enableContinuousInput(-180, 180);
    }

    private void findClosestAprilTag() {
        if (fieldLayout == null) return;

        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d robotTranslation = robotPose.getTranslation();
        
        double closestDistance = Double.MAX_VALUE;
        int closestId = -1;

        // Find the closest valid AprilTag
        for (var tagPose : fieldLayout.getTags()) {
            if (!VALID_TAG_IDS.contains(tagPose.ID)) continue;

            Translation2d tagTranslation = tagPose.pose.getTranslation().toTranslation2d();
            double distance = robotTranslation.getDistance(tagTranslation);
            
            if (distance < closestDistance) {
                closestDistance = distance;
                closestId = tagPose.ID;
            }
        }

        closestTagId = closestId;
    }

    private void updateTargetPosition() {
        if (fieldLayout == null || closestTagId == -1) return;

        var targetTag = fieldLayout.getTagPose(closestTagId);
        if (targetTag.isEmpty()) return;

        Pose3d tagPose = targetTag.get();
        
        double tagRotationDegrees = tagPose.getRotation().toRotation2d().getDegrees();
        double tagRotationRadians = Math.toRadians(tagRotationDegrees);

        double dx = relativeX * Math.cos(tagRotationRadians) - relativeY * Math.sin(tagRotationRadians);
        double dy = relativeX * Math.sin(tagRotationRadians) + relativeY * Math.cos(tagRotationRadians);

        targetX = tagPose.getX() - dx;
        targetY = tagPose.getY() - dy;
        targetYaw = tagRotationDegrees + (tagRotationDegrees > 0 ? -180 : 180);

        setSetpoint(targetX, targetY, targetYaw);
    }

    private void setSetpoint(double x, double y, double yaw) {
        xPidController.setSetpoint(x);
        yPidController.setSetpoint(y);
        yawPidController.setSetpoint(yaw);
    }
}