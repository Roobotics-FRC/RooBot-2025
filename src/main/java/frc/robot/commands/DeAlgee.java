package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;

public class DeAlgee extends Command{
    SuperstructureSubsystem superstructureSubsystem;
    LEDSubsystem ledSubsystem;
    double StPose;

    public DeAlgee(SuperstructureSubsystem m_SuperstructureSubsystem, double StPose, LEDSubsystem ledSubsystem) {
        this.superstructureSubsystem = m_SuperstructureSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.StPose = StPose;
    }

    @Override
    public void initialize() {
        superstructureSubsystem.setElevatorSpeed(1);
        superstructureSubsystem.de_algeefy();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return (int)superstructureSubsystem.position == (int)StPose || (int)superstructureSubsystem.position == (int)StPose - 1|| (int)superstructureSubsystem.position == (int)StPose + 1;
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        superstructureSubsystem.setElevatorSpeed(0);
        superstructureSubsystem.stopMotors();
        ledSubsystem.setTeleopMode();
    }
}