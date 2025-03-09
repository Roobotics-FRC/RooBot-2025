package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
public class LEDCommands {
    private final LEDSubsystem ledSubsystem;

    public LEDCommands(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
    }

    public Command autonomous() {
        return new SetLEDState(ledSubsystem, LEDState.BREATHING, Color.kGold);
    }

    public Command alignment() {
        return new SetLEDState(ledSubsystem, LEDState.BLINKING, Color.kGreen);
    }

    // Changed to SOLID state with alliance color
    public Command disabled() {
        return new SetLEDState(ledSubsystem, LEDState.SOLID, 
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? 
            Color.kRed : Color.kBlue);
    }

    // Changed to BREATHING state with alliance color
    public Command teleop() {
        return new SetLEDState(ledSubsystem, LEDState.BREATHING, 
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? 
            Color.kRed : Color.kBlue);
    }

    public Command intaking() {
        return new SetLEDState(ledSubsystem, LEDState.BLINKING, Color.kOrange);
    }

    public Command hasGamePiece() {
        return new SetLEDState(ledSubsystem, LEDState.SOLID, Color.kGreen);
    }

    public Command readyToScore() {
        return new SetLEDState(ledSubsystem, LEDState.BLINKING, Color.kPurple);
    }
}