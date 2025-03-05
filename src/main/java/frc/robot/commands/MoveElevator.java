package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;


public class MoveElevator extends Command{
    private final SuperstructureSubsystem superstructureSubsystem;
    private final double height;
    boolean velocity;

    public MoveElevator(SuperstructureSubsystem superstructureSubsystem, double height, boolean velocity) {
        this.superstructureSubsystem = superstructureSubsystem;
        this.height = height;
        this.velocity = velocity;

        addRequirements(superstructureSubsystem);
    }

    @Override
    public void initialize() {
        if (velocity){
            superstructureSubsystem.setElevatorSpeed(1);
        }
        else {
            superstructureSubsystem.setElevatorPosition(height);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if ((int)superstructureSubsystem.position == (int)height || (int)superstructureSubsystem.position == (int)height - 1|| (int)superstructureSubsystem.position == (int)height + 1){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        superstructureSubsystem.stopMotors();
    }
}
