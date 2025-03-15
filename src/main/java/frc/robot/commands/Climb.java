package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;

public class Climb extends Command{
    private final SuperstructureSubsystem superstructureSubsystem;
    private final double speed;

    public Climb(SuperstructureSubsystem superstructureSubsystem,double speed){
        this.superstructureSubsystem = superstructureSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        superstructureSubsystem.moveClimb(speed);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        superstructureSubsystem.moveClimb(0);
    }
}
