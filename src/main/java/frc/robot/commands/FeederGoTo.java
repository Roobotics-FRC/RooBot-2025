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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FeederGoTo extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final FieldCentric drive;
    private final double maxSpeed;
    private final double relativeX = Constants.Offsets.xFeeder;
    private final double relativeY = Constants.Offsets.yFeeder; 
    
    private double targetX;
    private double targetY;
    private double targetYaw;

    private AprilTagFieldLayout fieldLayout;
    private int closestTagId = -1;

    private static final Set<Integer> VALID_TAG_IDS = Set.of(
        1,2,
        13, 12
    );
    
    private final PIDController xPidController = new PIDController(Constants.PID.translationalKP, Constants.PID.translationalKI, Constants.PID.translationalKD);
    private final PIDController yPidController = new PIDController(Constants.PID.translationalKP, Constants.PID.translationalKI, Constants.PID.translationalKD);
    private final PIDController yawPidController = new PIDController(0.2, Constants.PID.rotationalKI, Constants.PID.rotationalKD);

    /**
     * Creates a new AutoGoTo command that goes to the closest valid AprilTag
     * @param drivetrain The swerve drivetrain subsystem
     * @param maxSpeed Maximum speed for movement
     * @param superstructureSubsystem The superstructure subsystem for LEDs
     */
    public FeederGoTo(CommandSwerveDrivetrain drivetrain, double maxSpeed) {
        this.drivetrain = drivetrain;
        this.maxSpeed = maxSpeed;
        this.drive = new SwerveRequest.FieldCentric();
        
        addRequirements(drivetrain);
        
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTag field layout", e.getStackTrace());
        }
        
        config();
    }

    @Override
    public void initialize() {
        resetPID();
        findClosestAprilTag();
        if (closestTagId != -1) {
            updateTargetPosition();
        }
    }

    @Override
    public void execute() {
        if (closestTagId == -1) {
            findClosestAprilTag();
            if (closestTagId != -1) {
                updateTargetPosition();
            }
            return;
        }

        Pose2d currentPose = drivetrain.getState().Pose;

        DriverStation.getAlliance().ifPresent(allianceColor -> {
            if (allianceColor == DriverStation.Alliance.Red) {
                drivetrain.setControl(
                    drive.withVelocityX(xPidController.calculate(currentPose.getX()) * -maxSpeed)
                        .withVelocityY(yPidController.calculate(currentPose.getY()) * -maxSpeed)
                        .withRotationalRate(yawPidController.calculate(currentPose.getRotation().getDegrees()))
                );
            } else {
                drivetrain.setControl(
                    drive.withVelocityX(xPidController.calculate(currentPose.getX()) * maxSpeed)
                        .withVelocityY(yPidController.calculate(currentPose.getY()) * maxSpeed)
                        .withRotationalRate(yawPidController.calculate(currentPose.getRotation().getDegrees()))
                );
            }
        });
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
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
        targetYaw = tagRotationDegrees;

        setSetpoint(targetX, targetY, targetYaw);
    }

    private void setSetpoint(double x, double y, double yaw) {
        xPidController.setSetpoint(x);
        yPidController.setSetpoint(y);
        yawPidController.setSetpoint(yaw);
    }
}