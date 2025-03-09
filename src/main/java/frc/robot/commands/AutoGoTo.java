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
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;

public class AutoGoTo extends Command {
    // State machine enum definition
    private enum AlignmentState {
        APPROACH,            // Initial approach - high speed, low precision
        FINE_ADJUSTMENT,     // Fine adjustment - low speed, high precision
        FINAL_ALIGNMENT,     // Final alignment - very low speed, very high precision
        HOLDING,             // Holding position
        COMPLETE             // Alignment complete
    }
    
    private AlignmentState currentState = AlignmentState.APPROACH;
    private int stableCount = 0;
    private final int requiredStableCycles = 10;
    
    // Original fields from your implementation
    private final CommandSwerveDrivetrain drivetrain;
    private final FieldCentric drive;
    private final LEDSubsystem ledSubsystem;
    private final double maxSpeed;
    private final double relativeX;
    private final double relativeY; 
    
    private double targetX;
    private double targetY;
    private double targetYaw;

    private AprilTagFieldLayout fieldLayout;
    private int closestTagId = -1;

    // Valid AprilTag IDs (6-11 and 17-22)
    private static final Set<Integer> VALID_TAG_IDS = Set.of(
        6, 7, 8, 9, 10, 11,
        17, 18, 19, 20, 21, 22
    );
    
    // PID Controllers with different tuning parameters for each state
    // Approach: High speed movement toward the target position
    private final PIDController xPidApproach = new PIDController(Constants.PID.translationalKP * 1.2, 0, Constants.PID.translationalKD * 0.8);
    private final PIDController yPidApproach = new PIDController(Constants.PID.translationalKP * 1.2, 0, Constants.PID.translationalKD * 0.8);
    private final PIDController yawPidApproach = new PIDController(0.25, 0, Constants.PID.rotationalKD * 0.8);
    
    // Fine: More balanced approach with some integral term
    private final PIDController xPidFine = new PIDController(Constants.PID.translationalKP * 0.8, Constants.PID.translationalKI * 0.5, Constants.PID.translationalKD);
    private final PIDController yPidFine = new PIDController(Constants.PID.translationalKP * 0.8, Constants.PID.translationalKI * 0.5, Constants.PID.translationalKD);
    private final PIDController yawPidFine = new PIDController(0.15, Constants.PID.rotationalKI * 0.5, Constants.PID.rotationalKD);
    
    // Final: Precision-focused with stronger integral component
    private final PIDController xPidFinal = new PIDController(Constants.PID.translationalKP * 0.5, Constants.PID.translationalKI, Constants.PID.translationalKD * 1.2);
    private final PIDController yPidFinal = new PIDController(Constants.PID.translationalKP * 0.5, Constants.PID.translationalKI, Constants.PID.translationalKD * 1.2);
    private final PIDController yawPidFinal = new PIDController(0.1, Constants.PID.rotationalKI, Constants.PID.rotationalKD * 1.2);
    
    // Which set of controllers is active
    private PIDController xPidController;
    private PIDController yPidController;
    private PIDController yawPidController;
    
    // Thresholds for state transitions
    private final double fineAdjustmentThreshold = 1; // meters
    private final double finalAlignmentThreshold = 0.2; // meters
    private final double holdingThreshold = 0.05; // meters
    private final double yawFineThreshold = 20.0; // degrees
    private final double yawFinalThreshold = 10.0; // degrees
    private final double yawHoldingThreshold = 3.0; // degrees

    private final double fineFactor = 1;
    private final double finalFactor = 0.6;

    /**
     * Creates a new AutoGoTo command that goes to the closest valid AprilTag
     * @param drivetrain The swerve drivetrain subsystem
     * @param maxSpeed Maximum speed for movement
     * @param relativeX Distance to maintain in X direction relative to the tag in meters
     * @param relativeY Distance to maintain in Y direction relative to the tag in meters
     * @param ledSubsystem The LED subsystem for visual feedback
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
        
        // Set the initial PID controller to approach controllers
        xPidController = xPidApproach;
        yPidController = yPidApproach;
        yawPidController = yawPidApproach;
        
        configAllPIDControllers();
    }

    @Override
    public void initialize() {
        LimelightHelpers.setLEDMode_ForceOn("limelight");
        resetAllPIDControllers();

        changeState(AlignmentState.APPROACH);
        stableCount = 0;
        
        findClosestAprilTag();
        if (closestTagId != -1) {
            updateTargetPosition();
        }
        
        // Log initial state
        SmartDashboard.putString("Alignment State", currentState.toString());
    }

    @Override
    public void execute() {
        // Get current pose
        Pose2d currentPose = drivetrain.getState().Pose;
        
        // Display current state on dashboard for debugging
        SmartDashboard.putString("Alignment State", currentState.toString());
        
        // State machine logic
        switch (currentState) {
            case APPROACH -> handleApproachState(currentPose);
                
            case FINE_ADJUSTMENT -> handleFineAdjustmentState(currentPose);
                
            case FINAL_ALIGNMENT -> handleFinalAlignmentState(currentPose);
                
            case HOLDING -> handleHoldingState(currentPose);
                
            case COMPLETE -> {
            }
        }
    }
    
    private void handleApproachState(Pose2d currentPose) {
        
        // Calculate translational and rotational errors
        double xError = Math.abs(targetX - currentPose.getX());
        double yError = Math.abs(targetY - currentPose.getY());
        double translationError = Math.hypot(xError, yError);
        double rotationError = Math.abs(angleDifference(targetYaw, currentPose.getRotation().getDegrees()));

        // Move using approach PID values (fast approach)
        moveRobot(currentPose, 1.0); // Full speed during approach
        
        // Check if we're close enough to switch to fine adjustment
        if (translationError < fineAdjustmentThreshold && rotationError < yawFineThreshold) {
            changeState(AlignmentState.FINE_ADJUSTMENT);
        }
    }
    
    private void handleFineAdjustmentState(Pose2d currentPose) {
        
        // Calculate errors
        double xError = Math.abs(targetX - currentPose.getX());
        double yError = Math.abs(targetY - currentPose.getY());
        double translationError = Math.hypot(xError, yError);
        double rotationError = Math.abs(angleDifference(targetYaw, currentPose.getRotation().getDegrees()));
        
        // Move using fine adjustment PID values (slower, more precise)
        moveRobot(currentPose, fineFactor);
        
        // Check if we're close enough to switch to final alignment
        if (translationError < finalAlignmentThreshold && rotationError < yawFinalThreshold) {
            changeState(AlignmentState.FINAL_ALIGNMENT);
        }
        
        // If we drift too far, go back to approach
        if (translationError > fineAdjustmentThreshold * 1.5 || rotationError > yawFineThreshold * 1.5) {
            changeState(AlignmentState.APPROACH);
        }
    }
    
    private void handleFinalAlignmentState(Pose2d currentPose) {
        // Calculate errors
        double xError = Math.abs(targetX - currentPose.getX());
        double yError = Math.abs(targetY - currentPose.getY());
        double translationError = Math.hypot(xError, yError);
        double rotationError = Math.abs(angleDifference(targetYaw, currentPose.getRotation().getDegrees()));
        
        // Move using final alignment PID values (very slow, very precise)
        moveRobot(currentPose, finalFactor);
        
        // Check if we're close enough to switch to holding
        if (translationError < holdingThreshold && rotationError < yawHoldingThreshold) {
            stableCount++;
            
            // If we've been stable for enough cycles, switch to holding
            if (stableCount >= requiredStableCycles) {
                changeState(AlignmentState.HOLDING);
            }
        } else {
            // Reset stable count if we drift
            stableCount = 0;
        }
        
        // If we drift too far, go back to fine adjustment
        if (translationError > finalAlignmentThreshold * 1.5 || rotationError > yawFinalThreshold * 1.5) {
            changeState(AlignmentState.FINE_ADJUSTMENT);
        }
    }
    
    private void handleHoldingState(Pose2d currentPose) {
        
        // Calculate errors
        double xError = Math.abs(targetX - currentPose.getX());
        double yError = Math.abs(targetY - currentPose.getY());
        double translationError = Math.hypot(xError, yError);
        double rotationError = Math.abs(angleDifference(targetYaw, currentPose.getRotation().getDegrees()));
        
        // Move using final alignment PID values but at even lower speed
        moveRobot(currentPose, 0.2); // 20% speed during holding
        
        // Check if we have held position long enough
        if (translationError < holdingThreshold && rotationError < yawHoldingThreshold) {
            stableCount++;
            
            // If we've been stable for long enough, complete
            if (stableCount > requiredStableCycles * 2) {
                changeState(AlignmentState.COMPLETE);
            }
        } else {
            // Reset stable count if we drift too much
            if (translationError > holdingThreshold * 1.5 || rotationError > yawHoldingThreshold * 1.5) {
                stableCount = 0;
            }
        }
        
        // If we drift too far, go back to final alignment
        if (translationError > finalAlignmentThreshold || rotationError > yawFinalThreshold) {
            changeState(AlignmentState.FINAL_ALIGNMENT);
        }
    }
    
    private void moveRobot(Pose2d currentPose, double speedFactor) {
        // Calculate PID outputs
        double xOutput = xPidController.calculate(currentPose.getX());
        double yOutput = yPidController.calculate(currentPose.getY());
        double yawOutput = yawPidController.calculate(currentPose.getRotation().getDegrees());
        
        // Apply speed factor based on state
        double adjustedMaxSpeed = maxSpeed * speedFactor;
        
        // Apply minimum output if we're close but not quite there (overcome static friction)
        double minOutput = 0.02; // Minimum output to overcome static friction
        if (Math.abs(xOutput) < minOutput && !xPidController.atSetpoint()) {
            xOutput = Math.copySign(minOutput, xOutput);
        }
        if (Math.abs(yOutput) < minOutput && !yPidController.atSetpoint()) {
            yOutput = Math.copySign(minOutput, yOutput);
        }
        
        // Apply minimum rotational output if needed
        double minRotOutput = 0.01;
        if (Math.abs(yawOutput) < minRotOutput && !yawPidController.atSetpoint()) {
            yawOutput = Math.copySign(minRotOutput, yawOutput);
        }
        
        // Debugging
        SmartDashboard.putNumber("xOutput", xOutput);
        SmartDashboard.putNumber("yOutput", yOutput);
        SmartDashboard.putNumber("yawOutput", yawOutput);
        
        // Send commands to drivetrain
        drivetrain.setControl(
            drive.withVelocityX(xOutput * adjustedMaxSpeed)
                 .withVelocityY(yOutput * adjustedMaxSpeed)
                 .withRotationalRate(yawOutput)
        );
    }
    
    private void changeState(AlignmentState newState) {
        if (newState != currentState) {
            
            // Change active PID controllers based on state
            switch (newState) {
                case APPROACH -> {
                    xPidController = xPidApproach;
                    yPidController = yPidApproach;
                    yawPidController = yawPidApproach;
                }
                    
                case FINE_ADJUSTMENT -> {
                    xPidController = xPidFine;
                    yPidController = yPidFine;
                    yawPidController = yawPidFine;
                }
                    
                case FINAL_ALIGNMENT -> {
                    xPidController = xPidFinal;
                    yPidController = yPidFinal;
                    yawPidController = yawPidFinal;
                }
                    
                case HOLDING, COMPLETE -> {
                }

            }
            
            // Reset integral terms when changing states to prevent windup
            if (newState != AlignmentState.HOLDING && newState != AlignmentState.COMPLETE) {
                xPidController.reset();
                yPidController.reset();
                yawPidController.reset();
                
                // Set setpoints
                if (closestTagId != -1) {
                    xPidController.setSetpoint(targetX);
                    yPidController.setSetpoint(targetY);
                    yawPidController.setSetpoint(targetYaw);
                }
            }
            
            // Update the current state
            currentState = newState;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        // Add this to ensure we return to alliance color even if interrupted
        ledSubsystem.setTeleopMode();
        LimelightHelpers.setLEDMode_ForceOff("limelight");
        
        System.out.println("AutoGoTo: Command " + (interrupted ? "interrupted" : "completed successfully"));
    }

    @Override
    public boolean isFinished() {
        return currentState == AlignmentState.COMPLETE;
    }

    private void resetAllPIDControllers() {
        // Reset all PID controllers
        xPidApproach.reset();
        yPidApproach.reset();
        yawPidApproach.reset();
        
        xPidFine.reset();
        yPidFine.reset();
        yawPidFine.reset();
        
        xPidFinal.reset();
        yPidFinal.reset();
        yawPidFinal.reset();
    }

    private void configAllPIDControllers() {
        // Configure Approach PIDs
        xPidApproach.setIZone(Constants.PID.thanslationalIZone);
        yPidApproach.setIZone(Constants.PID.thanslationalIZone);
        xPidApproach.setTolerance(fineAdjustmentThreshold);
        yPidApproach.setTolerance(fineAdjustmentThreshold);
        yawPidApproach.setTolerance(yawFineThreshold);
        yawPidApproach.enableContinuousInput(-180, 180);

        xPidFine.setTolerance(finalAlignmentThreshold);
        yPidFine.setTolerance(finalAlignmentThreshold);
        yawPidFine.setTolerance(yawFinalThreshold);
        yawPidFine.enableContinuousInput(-180, 180);
        
        xPidFinal.setTolerance(holdingThreshold);
        yPidFinal.setTolerance(holdingThreshold);
        yawPidFinal.setTolerance(yawHoldingThreshold);
        yawPidFinal.enableContinuousInput(-180, 180);
    }

    // Your original helper methods
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
        
        // Also update other controllers to ensure smooth transitions
        xPidApproach.setSetpoint(x);
        yPidApproach.setSetpoint(y);
        yawPidApproach.setSetpoint(yaw);
        
        xPidFine.setSetpoint(x);
        yPidFine.setSetpoint(y);
        yawPidFine.setSetpoint(yaw);
        
        xPidFinal.setSetpoint(x);
        yPidFinal.setSetpoint(y);
        yawPidFinal.setSetpoint(yaw);
    }
    
    // Helper to calculate the shortest angular distance
    private double angleDifference(double target, double current) {
        double diff = target - current;
        // Normalize to -180 to 180
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }
}