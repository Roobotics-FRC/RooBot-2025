package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;

public class AutoDeAlgee extends Command{
    SuperstructureSubsystem superstructureSubsystem;
    double StPose;

    public AutoDeAlgee(SuperstructureSubsystem m_SuperstructureSubsystem, double StPose) {
        this.superstructureSubsystem = m_SuperstructureSubsystem;
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
    }
}