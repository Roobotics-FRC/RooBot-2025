package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;

public class MoveHoper extends Command{
    SuperstructureSubsystem superstructureSubsystem;
    double pose;
    boolean limit;

    public MoveHoper(SuperstructureSubsystem superstructureSubsystem, double pose, boolean limit) {
        this.superstructureSubsystem = superstructureSubsystem;
        this.pose = pose;
        this.limit = limit;

        addRequirements(superstructureSubsystem);
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
        if(limit == false){
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
