package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;

public class DeAlgee extends Command{
    SuperstructureSubsystem superstructureSubsystem;
    double StPose;

    public DeAlgee(SuperstructureSubsystem superstructureSubsystem, double StPose) {
        this.superstructureSubsystem = superstructureSubsystem;
        this.StPose = StPose;

        addRequirements(superstructureSubsystem);
    }

    @Override
    public void initialize() {
        superstructureSubsystem.setElevatorSpeed(1.5);
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