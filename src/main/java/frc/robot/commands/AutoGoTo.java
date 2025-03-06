package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperstructureSubsystem;

public class AutoGoTo extends Command {
    private double ID;
    private final boolean RightLeft;
    
    //!Positions
    private final Pose2d ID17R = new Pose2d(new Translation2d(3.980, 2.810), new Rotation2d(Math.toDegrees(60)));
    private final Pose2d ID17L = new Pose2d(new Translation2d(3.685, 2.975), new Rotation2d(Math.toDegrees(120)));
    private final Pose2d ID18R = new Pose2d(new Translation2d(3.15, 3.95), new Rotation2d(Math.toDegrees(0)));
    private final Pose2d ID18L = new Pose2d(new Translation2d(3.15, 4.195), new Rotation2d(Math.toDegrees(0)));
    private final Pose2d ID19R = new Pose2d(new Translation2d(3.685, 5.08), new Rotation2d(Math.toDegrees(300)));
    private final Pose2d ID19L = new Pose2d(new Translation2d(3.975, 5.252), new Rotation2d(Math.toDegrees(300)));
    private final Pose2d ID20R = new Pose2d(new Translation2d(4.99, 5.25), new Rotation2d(Math.toDegrees(240)));
    private final Pose2d ID20L = new Pose2d(new Translation2d(5.28, 5.08), new Rotation2d(Math.toDegrees(240)));
    private final Pose2d ID21R = new Pose2d(new Translation2d(5.8, 4.195), new Rotation2d(Math.toDegrees(180)));
    private final Pose2d ID21L = new Pose2d(new Translation2d(5.790, 3.865), new Rotation2d(Math.toDegrees(180)));
    private final Pose2d ID22R = new Pose2d(new Translation2d(5.285, 2.972), new Rotation2d(Math.toDegrees(120)));
    private final Pose2d ID22L = new Pose2d(new Translation2d(4.995, 2.810), new Rotation2d(Math.toDegrees(120)));
    
    private final CommandSwerveDrivetrain drivetrain;
    private final SuperstructureSubsystem superstructureSubsystem;
    private final FieldCentric drive;
    private final double maxSpeed;

    double poseX;
    double poseY;

    private double x = 0;
    private double y = 0;
    private double yaw = 0;

    private final PIDController xPidController = new PIDController(Constants.PID.translationalKP, Constants.PID.translationalKI, Constants.PID.translationalKD);
    private final PIDController yPidController = new PIDController(Constants.PID.translationalKP, Constants.PID.translationalKI, Constants.PID.translationalKD);
    private final PIDController yawPidController = new PIDController(Constants.PID.rotationalKP, Constants.PID.rotationalKI, Constants.PID.rotationalKD);

    NetworkTable driveStateTable = NetworkTableInstance.getDefault().getTable("DriveState");
    StructSubscriber<Pose2d> poseSubscriber = driveStateTable.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d());

    /**
     * Constructs a new AutoGoTo command for autonomous robot navigation.
     * @param RightLeft Boolean flag indicating right (true) or left (false) side movement preference
     */
    public AutoGoTo(CommandSwerveDrivetrain drivetrain, double maxSpeed, SuperstructureSubsystem superstructureSubsystem, boolean RightLeft) {
        this.drivetrain = drivetrain;
        this.superstructureSubsystem = superstructureSubsystem;
        this.maxSpeed = maxSpeed;
        this.RightLeft = RightLeft;
        
        this.drive = new SwerveRequest.FieldCentric();
        
        addRequirements(drivetrain);

        config();

        //!FOR NOW
        ID = 18;
    }

    @Override
    public void initialize() {
        superstructureSubsystem.setLED(Color.kGreen, 0.2, 0.1);
        resetPID();
        getSetpoint();
        setSetpoint(x, y, yaw);
    }

    @Override
    public void execute() {
        poseX = poseSubscriber.get().getX();
        poseY = poseSubscriber.get().getY();

        drivetrain.setControl(
            drive.withVelocityX(xPidController.calculate(poseX) * maxSpeed)
                .withVelocityY(yPidController.calculate(poseY) * maxSpeed)
                .withRotationalRate(yawPidController.calculate(poseSubscriber.get().getRotation().getDegrees()))
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        superstructureSubsystem.setLED(Color.kBlue, 0, 0);
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

    private void setSetpoint(double x, double y, double yaw) {
        xPidController.setSetpoint(x);
        yPidController.setSetpoint(y);
        yawPidController.setSetpoint(yaw);
    }

    private void getSetpoint() {
        if(RightLeft){
            if(ID == 17){
                x = ID17R.getX();
                y = ID17R.getY();
                yaw = ID17R.getRotation().getDegrees();
            } else if(ID == 18){
                x = ID18R.getX();
                y = ID18R.getY();
                yaw = ID18R.getRotation().getDegrees();
            } else if(ID == 19){
                x = ID19R.getX();
                y = ID19R.getY();
                yaw = ID19R.getRotation().getDegrees();
            } else if(ID == 20){
                x = ID20R.getX();
                y = ID20R.getY();
                yaw = ID20R.getRotation().getDegrees();
            } else if(ID == 21){
                x = ID21R.getX();
                y = ID21R.getY();
                yaw = ID21R.getRotation().getDegrees();
            } else if(ID == 22){
                x = ID22R.getX();
                y = ID22R.getY();
                yaw = ID22R.getRotation().getDegrees();
            }
        } else {
            if(ID == 17){
                x = ID17L.getX();
                y = ID17L.getY();
                yaw = ID17L.getRotation().getDegrees();
            } else if(ID == 18){
                x = ID18L.getX();
                y = ID18L.getY();
                yaw = ID18L.getRotation().getDegrees();
            } else if(ID == 19){
                x = ID19L.getX();
                y = ID19L.getY();
                yaw = ID19L.getRotation().getDegrees();
            } else if(ID == 20){
                x = ID20L.getX();
                y = ID20L.getY();
                yaw = ID20L.getRotation().getDegrees();
            } else if(ID == 21){
                x = ID21L.getX();
                y = ID21L.getY();
                yaw = ID21L.getRotation().getDegrees();
            } else if(ID == 22){
                x = ID22L.getX();
                y = ID22L.getY();
                yaw = ID22L.getRotation().getDegrees();
            }
        }
    }
}
