package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;

public class OutTake extends Command{
    SuperstructureSubsystem superstructureSubsystem;

    public OutTake(SuperstructureSubsystem m_SuperstructureSubsystem) {
        this.superstructureSubsystem = m_SuperstructureSubsystem;
    }

    @Override
    public void initialize() {
        superstructureSubsystem.outTake();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
