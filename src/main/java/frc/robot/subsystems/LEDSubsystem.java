package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final int ledCount;

    private LEDState currentState = LEDState.OFF;
    private Color currentColor = Color.kBlack;
    private double breathingValue = 0;
    private boolean breathingUp = true;
    private double time = 0;
    
    private final LEDPattern m_rainbow;
    private final LEDPattern m_scrollingRainbow;

    public enum LEDState {
        OFF,
        SOLID,
        BREATHING,
        BLINKING,
        RAINBOW,
        CHASE
    }

    public LEDSubsystem(int pwmPort, int ledCount) {
        this.ledCount = ledCount;
        m_rainbow = LEDPattern.rainbow(255, ledCount);
        m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1/120));
        ledStrip = new AddressableLED(pwmPort);
        ledBuffer = new AddressableLEDBuffer(ledCount);
        
        ledStrip.setLength(ledCount);
        ledStrip.start();
        setDisabledMode();
    }

    @Override
    public void periodic() {
        time += 0.02; // Assuming 50Hz refresh rate

        switch (currentState) {
            case BREATHING -> updateBreathing();
            case BLINKING -> updateBlinking();
            case RAINBOW -> updateRainbow();
            case CHASE -> updateChase();
            case SOLID -> setSolidColor(currentColor);
            case OFF -> setSolidColor(Color.kBlack);
        }

        ledStrip.setData(ledBuffer);
    }

    private void updateBreathing() {
        if (breathingUp) {
            breathingValue += 0.05;
            if (breathingValue >= 1.0) {
                breathingValue = 1.0;
                breathingUp = false;
            }
        } else {
            breathingValue -= 0.05;
            if (breathingValue <= 0.0) {
                breathingValue = 0.0;
                breathingUp = true;
            }
        }

        Color dimmedColor = new Color(
            currentColor.red * breathingValue,
            currentColor.green * breathingValue,
            currentColor.blue * breathingValue
        );
        setSolidColor(dimmedColor);
    }

    private void updateBlinking() {
        boolean isOn = ((int)(time * 2) % 2) == 0;
        setSolidColor(isOn ? currentColor : Color.kBlack);
    }

    private void updateRainbow() {
        for (int i = 0; i < ledCount; i++) {
            final int hue = (int)((time * 50 + (i * 180.0 / ledCount)) % 180);
            ledBuffer.setHSV(i, hue, 255, 128);
        }
    }
    
    private void updateChase() {
        int position = (int)(time * 10) % ledCount;
        for (int i = 0; i < ledCount; i++) {
            if (i == position) {
                ledBuffer.setLED(i, currentColor);
            } else {
                ledBuffer.setLED(i, Color.kBlack);
            }
        }
    }

    public void setSolidColor(Color color) {
        for (int i = 0; i < ledCount; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public void setLEDState(LEDState state, Color color) {
        currentState = state;
        currentColor = color;
        breathingValue = 0;
        breathingUp = true;
        time = 0;
    }

    public void setAutonomousMode() {
        setLEDState(LEDState.BREATHING, Color.kGold);
    }

    public void setAlignmentMode() {
        setLEDState(LEDState.BLINKING, Color.kGreen);
    }

    public void setDisabledMode() {
        if (DriverStation.getAlliance().isPresent()) {
            Color allianceColor = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 
                Color.kRed : Color.kBlue;
            setLEDState(LEDState.SOLID, allianceColor);
        } else {
            setLEDState(LEDState.SOLID, Color.kWhite);
        }
    }

    public void setTeleopMode() {
        if (DriverStation.getAlliance().isPresent()) {
            Color allianceColor = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 
                Color.kRed : Color.kBlue;
            setLEDState(LEDState.BREATHING, allianceColor);
        } else {
            setLEDState(LEDState.BREATHING, Color.kWhite);
        }
    }

    public void setIntaking() {
        setLEDState(LEDState.CHASE, Color.kGreen);
    }

    public void setHasGamePiece() {
        setLEDState(LEDState.SOLID, Color.kGreen);
    }

    public void setReadyToScore() {
        setLEDState(LEDState.BLINKING, Color.kPurple);
    }
}