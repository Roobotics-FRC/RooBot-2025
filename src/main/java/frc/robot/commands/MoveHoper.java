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
        if (superstructureSubsystem.PieceIn){
            superstructureSubsystem.setIntakeSpeed(-0.6);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if(pose == -3){
            return true;
        }else{
            return !superstructureSubsystem.PieceIn;
        }
    }

    @Override
    public void end(boolean interrupted) {
        superstructureSubsystem.setIntakeSpeed(0);
    }
}
