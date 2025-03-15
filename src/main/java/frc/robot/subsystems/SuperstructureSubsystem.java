package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperstructureSubsystem extends SubsystemBase {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    private final TalonFX L_elevatorMotor = new TalonFX(21);
    private final TalonFX R_elevatorMotor = new TalonFX(22);

    private final TalonFX ClimbMotor = new TalonFX(23);

    private final double ClimbP = 0.3;

    private final double ElevatorP = 1;
    private final double ElevatorI = 0;
    private final double ElevatorD = 0;
    private double ClimbPosition = 0;

    public double position;

    private final SparkMax intakeMotor = new SparkMax(32, SparkMax.MotorType.kBrushless);
    private final SparkMax de_algee = new SparkMax(33, SparkMax.MotorType.kBrushless);
    private final SparkMax hoper = new SparkMax(31, SparkMax.MotorType.kBrushless);

    private final Joystick controller;

    public SuperstructureSubsystem(Joystick controller) {
        this.controller = controller;
        configElevator();

        L_elevatorMotor.setPosition(0);
        R_elevatorMotor.setPosition(0);

    }

    private void configElevator() {
        TalonFXConfiguration climbConfig = new TalonFXConfiguration();
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        
        elevatorConfig.Slot0.kP = ElevatorP;
        elevatorConfig.Slot0.kI = ElevatorI;
        elevatorConfig.Slot0.kD = ElevatorD;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = 12;
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        feedbackConfigs.RotorToSensorRatio = 1;

        climbConfig.Slot0.kP = ClimbP;
        climbConfig.Slot0.kI = 0;
        climbConfig.Slot0.kD = 0;
        climbConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        L_elevatorMotor.getConfigurator().apply(elevatorConfig);
        R_elevatorMotor.getConfigurator().apply(elevatorConfig);
        L_elevatorMotor.getConfigurator().apply(feedbackConfigs);
        R_elevatorMotor.getConfigurator().apply(feedbackConfigs);
        ClimbMotor.getConfigurator().apply(climbConfig);
    }

    public void setElevatorPosition(double position) {
        L_elevatorMotor.setControl(m_request.withPosition(position));
        R_elevatorMotor.setControl(m_request.withPosition(position));
    }

    public void setElevatorSpeed(double speed){
        L_elevatorMotor.setControl(new VoltageOut(speed));
        R_elevatorMotor.setControl(new VoltageOut(speed));
    }

    public double getElevatorPosition() {
        position = L_elevatorMotor.getPosition().getValueAsDouble();
        return position;
    }

    public void outTake(){
        intakeMotor.set(-0.8);
    }

    public void stopMotors(){
        intakeMotor.stopMotor();
        de_algee.stopMotor();
    }

    public void de_algeefy(){
        de_algee.set(-1);
    }

    public void moveHoper(double poseition){
        hoper.getClosedLoopController().setReference(poseition,ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void moveClimb(double position){
        ClimbMotor.setControl(new VoltageOut(position));
    }

    @Override
    public void periodic() {
        getElevatorPosition();

        // double ID = LimelightHelpers.getFiducialID("limelight");
        // SmartDashboard.putNumber("ID", ID);

        // //! Hand Controll elevator height
        // if (controller.getRawAxis(3) != 1){
        //     setElevatorPosition((((controller.getRawAxis(3)*-1)+1)/2)*70);
        //     System.out.println(getElevatorPosition());
        // }
    }
}