package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// setup/constants

public class ElevatorSubsystem extends SubsystemBase {

    //TODO: find ids

    private final TalonFX L_elevatorMotor = new TalonFX(0);
    private final TalonFX R_elevatorMotor = new TalonFX(1);

    private final double MAX_VELOCITY = 0;
    private final double MAX_ACCELERATION = 0;

    private double elevatorPosition = 0;
    private double ElevatorP = 0;
    private double ElevatorI = 0;
    private double ElevatorD = 0;
   
    

    public ElevatorSubsystem() {
        System.out.println("constructor called.");
        configElevator();
    }

    public void configElevator() {
        var leftConfiguration = new TalonFXConfiguration();
        var rightConfiguration = new TalonFXConfiguration();
        
        // TODO: find out inverted or not  ... also added max velocity & max acceleration
        
        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        L_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        R_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    
        leftConfiguration.Slot0.kP = ElevatorP;
        leftConfiguration.Slot0.kI = ElevatorI;
        leftConfiguration.Slot0.kD = ElevatorD;    

        L_elevatorMotor.getConfigurator().apply(leftConfiguration);
        R_elevatorMotor.getConfigurator().apply(rightConfiguration);

        leftConfiguration.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        leftConfiguration.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;
    
    }

    public double getElevatorPosition() {
        return elevatorPosition;
    }

    @Override
    public void periodic() {

    }
}