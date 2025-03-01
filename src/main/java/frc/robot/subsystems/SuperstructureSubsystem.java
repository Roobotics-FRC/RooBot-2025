package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class SuperstructureSubsystem extends SubsystemBase {

    private final TalonFX L_elevatorMotor = new TalonFX(1,"rio");
    private final TalonFX R_elevatorMotor = new TalonFX(2, "rio");

    public final double ElevatorP = 0.01;
    public final double ElevatorI = 0;
    public final double ElevatorD = 0;

   

    public SuperstructureSubsystem() {
        configElevator();
    }

    public void configElevator() {
        var slot0Configs = new Slot0Configs();
        FeedbackConfigs L_feedbackConfigs = new FeedbackConfigs();

        R_elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
        L_elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    
        slot0Configs.kP = ElevatorP;
        slot0Configs.kI = ElevatorI;
        slot0Configs.kD = ElevatorD;

        L_feedbackConfigs.RotorToSensorRatio = 1;

        L_elevatorMotor.getConfigurator().apply(slot0Configs);
        R_elevatorMotor.getConfigurator().apply(slot0Configs);
        L_elevatorMotor.getConfigurator().apply(L_feedbackConfigs);

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
        System.out.println(L_elevatorMotor.isConnected());
    }
}