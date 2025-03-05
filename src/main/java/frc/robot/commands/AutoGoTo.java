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
    private final Pose2d ID18R = new Pose2d(new Translation2d(3.15, 3.95), new Rotation2d(Math.toDegrees(0)));
    private final Pose2d ID18L = new Pose2d(new Translation2d(3.15, 3.95), new Rotation2d(Math.toDegrees(0)));
    private final Pose2d ID19R = new Pose2d(new Translation2d(3.15, 3.95), new Rotation2d(Math.toDegrees(0)));
    private final Pose2d ID19L = new Pose2d(new Translation2d(3.15, 3.95), new Rotation2d(Math.toDegrees(0)));


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
            if(ID == 18){
                x = ID18R.getX();
                y = ID18R.getY();
                yaw = ID18R.getRotation().getDegrees();
            } else if(ID == 19){
                x = ID19R.getX();
                y = ID19R.getY();
                yaw = ID19R.getRotation().getDegrees();
            }
        } else {
            if(ID == 18){
                x = ID18L.getX();
                y = ID18L.getY();
                yaw = ID18L.getRotation().getDegrees();
            } else if(ID == 19){
                x = ID19L.getX();
                y = ID19L.getY();
                yaw = ID19L.getRotation().getDegrees();
            }
        }
    }
}
