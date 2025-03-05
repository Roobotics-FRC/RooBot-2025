package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;

public class MoveHoper extends Command{
    SuperstructureSubsystem superstructureSubsystem;
    double pose;

    public MoveHoper(SuperstructureSubsystem m_SuperstructureSubsystem, double pose) {
        this.superstructureSubsystem = m_SuperstructureSubsystem;
        this.pose = pose;
    }

    @Override
    public void initialize() {
        superstructureSubsystem.moveHoper(pose);
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
