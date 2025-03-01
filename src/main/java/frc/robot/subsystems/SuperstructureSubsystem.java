package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperstructureSubsystem extends SubsystemBase {

    //TODO: find ids

    private final TalonFX L_elevatorMotor = new TalonFX(25);
    private final TalonFX R_elevatorMotor = new TalonFX(24);

    private final double MAX_VELOCITY = 0;
    private final double MAX_ACCELERATION = 0;

    private double ElevatorP = 0.01;
    private double ElevatorI = 0;
    private double ElevatorD = 0;

    private final MotionMagicDutyCycle motionMagicDutyCycle = new MotionMagicDutyCycle(0);
   

    public SuperstructureSubsystem() {
        configElevator();
    }

    public void configElevator() {
        TalonFXConfiguration leftConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration rightConfiguration = new TalonFXConfiguration();
        FeedbackConfigs L_feedbackConfigs = new FeedbackConfigs();
        
        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        L_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        R_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    
        leftConfiguration.Slot0.kP = ElevatorP;
        leftConfiguration.Slot0.kI = ElevatorI;
        leftConfiguration.Slot0.kD = ElevatorD;

        L_feedbackConfigs.RotorToSensorRatio = 1; //! YOUYR GEAR RATIO
        //TODOL: FIND THE PROPER ONE AND SET IOT IN THERE ^^^^^^
        // WE WANT THE BIKINI BOTTOM TO BE 0 AND TOP 1000 BADABING BADABOOM
        // MAX WILL BE 3450122
        // 1000/3450122  = 0.0002898 <- this is the ratio
        // 0.0002898 * 3450122 = 1000 <- this is the max
        

        L_elevatorMotor.getConfigurator().apply(leftConfiguration);
        R_elevatorMotor.getConfigurator().apply(rightConfiguration);
        L_elevatorMotor.getConfigurator().apply(L_feedbackConfigs);

        leftConfiguration.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        leftConfiguration.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;

        R_elevatorMotor.setControl(new com.ctre.phoenix6.controls.Follower(L_elevatorMotor.getDeviceID(), true));
    
    }

    public void setElevatorPosition(double position) {
        L_elevatorMotor.setPosition(position);
    }

    public double getElevatorPosition() {
        return L_elevatorMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        System.out.println("Elevator Position: " + getElevatorPosition());
        // setElevatorPosition(1000);
    }
}