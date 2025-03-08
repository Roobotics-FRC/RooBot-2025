package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class SetLEDState extends Command {
    private final LEDSubsystem ledSubsystem;
    private final LEDState state;
    private final Color color;

    public SetLEDState(LEDSubsystem ledSubsystem, LEDState state, Color color) {
        this.ledSubsystem = ledSubsystem;
        this.state = state;
        this.color = color;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.setLEDState(state, color);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}