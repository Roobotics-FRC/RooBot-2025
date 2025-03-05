package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class SuperstructureSubsystem extends SubsystemBase {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    private final TalonFX L_elevatorMotor = new TalonFX(21);
    private final TalonFX R_elevatorMotor = new TalonFX(22);

    private final TalonFX ClimbMotor = new TalonFX(23);

    private final double ClimbP = 1;

    private final double ElevatorP = 1;
    private final double ElevatorI = 0;
    private final double ElevatorD = 0;

    public double position;

    private final SparkMax intakeMotor = new SparkMax(32, SparkMax.MotorType.kBrushless);
    private final SparkMax de_algee = new SparkMax(33, SparkMax.MotorType.kBrushless);
    private final SparkMax hoper = new SparkMax(31, SparkMax.MotorType.kBrushless);

    private final Joystick controller;

    //! LED THINGY
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    public SuperstructureSubsystem(Joystick controller) {
        this.controller = controller;
        configElevator();

        L_elevatorMotor.setPosition(0);
        R_elevatorMotor.setPosition(0);


        //!LED
        m_led = new AddressableLED(0); // replace with PWM port
        m_led.setLength(60);
        m_led.setData(m_ledBuffer = new AddressableLEDBuffer(60)); // replace with proper length ?
        m_led.setData(m_ledBuffer); // apply buffer 
        m_led.start();

        //!Do we need this?
        // try (PWM led = new PWM(0)) {
        //     led.setPulseTimeMicroseconds(1500);
        // }

        setLED(Color.kBlue, 0,0);
    }

    private void configElevator() {
        TalonFXConfiguration climbConfig = new TalonFXConfiguration();
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        
        elevatorConfig.Slot0.kP = ElevatorP;
        elevatorConfig.Slot0.kI = ElevatorI;
        elevatorConfig.Slot0.kD = ElevatorD;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = 10;
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        feedbackConfigs.RotorToSensorRatio = 1;

        climbConfig.Slot0.kP = ClimbP;
        climbConfig.Slot0.kI = 0;
        climbConfig.Slot0.kD = 0;
        climbConfig.CurrentLimits.SupplyCurrentLimit = 100;
        climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        L_elevatorMotor.getConfigurator().apply(elevatorConfig);
        R_elevatorMotor.getConfigurator().apply(elevatorConfig);
        L_elevatorMotor.getConfigurator().apply(feedbackConfigs);
        R_elevatorMotor.getConfigurator().apply(feedbackConfigs);
        ClimbMotor.getConfigurator().apply(climbConfig);
    }
    
    /**
     * Sets the LED strip to display a static color pattern with optional blinking functionality.
     *
     * @param color The Color object representing the desired LED color
     * @param time_on Duration in milliseconds that the LED stays on during the pattern
     * @param time_off Duration in milliseconds that the LED stays off during the pattern
     */
    public void setLED(Color color, double time_on, double time_off) {
        if (time_on == 0 && time_off == 0) {
            LEDPattern acolor = LEDPattern.solid(color);
            acolor.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
        } else {
            LEDPattern acolor = LEDPattern.solid(color);
            acolor.blink(Seconds.of(time_on), Seconds.of(time_on));
            acolor.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
        }
    }

    public void setElevatorPosition(double position) {
        L_elevatorMotor.setControl(m_request.withPosition(position));
        R_elevatorMotor.setControl(m_request.withPosition(position));
    }

    public void setElevatorSpeed(double speed){
        L_elevatorMotor.setControl(new VoltageOut(speed));
        R_elevatorMotor.setControl(new VoltageOut(speed));
    }

    public double getElevatorPosition() {
        position = L_elevatorMotor.getPosition().getValueAsDouble();
        return position;
    }

    public void outTake(){
        intakeMotor.set(-1);
    }

    public void stopMotors(){
        intakeMotor.stopMotor();
        de_algee.stopMotor();
    }

    public void de_algeefy(){
        de_algee.set(-1);
    }

    public void moveHoper(double poseition){
        hoper.getClosedLoopController().setReference(poseition,ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void climbDown(){
        ClimbMotor.setControl(new PositionVoltage(0));
    }

    public void climbUp(int position){
        ClimbMotor.setControl(new PositionVoltage(position));
    }

    @Override
    public void periodic() {
        getElevatorPosition();

        double closestFiducial = LimelightHelpers.getFiducialID("limelight");

        SmartDashboard.putNumber("Closest Fiducial ID: ", closestFiducial);

        //! Hand Controll elevator height
        // if (controller.getRawAxis(3) != 1){
        //     setElevatorPosition((((controller.getRawAxis(3)*-1)+1)/2)*50);
        //     System.out.println((((controller.getRawAxis(3)*-1)+1)/2)*50);
        // }
    }
}