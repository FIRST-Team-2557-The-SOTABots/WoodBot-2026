package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Configs;

public class Climber extends SubsystemBase {
    private SparkFlex leftClimber;
    private SparkFlex rightClimber;

    private DigitalInput leftHallSensor;
    private DigitalInput rightHallSensor;

    private SparkFlexConfig leftClimberConfig;
    private SparkFlexConfig rightClimberConfig;

    private PIDController positionPid;
    private double targetPosition = 0.0;
    private boolean positionControlEnabled = false;

    // Sensor Status
    private boolean leftHallSensorTriggered = false;
    private boolean rightHallSensorTriggered = false;

    private final Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    private double m_startTime = 0;

    public Climber() {
        leftHallSensor = new DigitalInput(0);
        rightHallSensor = new DigitalInput(1);

        leftClimber = new SparkFlex(
                Constants.ClimberConstants.kClimberLeftCanId,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
        );
        rightClimber = new SparkFlex(
                Constants.ClimberConstants.kClimberRightCanId,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
        );

        leftClimberConfig = Configs.ClimberConfigs.leftClimberConfig;
        rightClimberConfig = Configs.ClimberConfigs.rightClimberConfig;

        leftClimber.configure(leftClimberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightClimber.configure(rightClimberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        isLeftHallSensorTriggered();
        isRightHallSensorTriggered();
        double timeRunning = Timer.getFPGATimestamp() - m_startTime;
        if (timeRunning > 0.5 && getCurrent() > 20.0 && leftClimber.get() < 0) {
            stop();
        }
    }

    public double getCurrent() {
        double leftCurrent = leftClimber.getOutputCurrent();
        double rightCurrent = rightClimber.getOutputCurrent();
        return (leftCurrent + rightCurrent) / 2.0;
    }

    // Manual voltage control
    public void setVoltage(double voltage) {
        // Record start time to bypass the periodic kill-switch during the initial spike
        if (voltage != 0 && leftClimber.getAppliedOutput() == 0) {
            m_startTime = Timer.getFPGATimestamp();
        }
        leftClimber.setVoltage(voltage);
        rightClimber.setVoltage(voltage);
    }

    // Encoder (never re-zeroed in match)
    public void zeroEncoders() {
        try { leftClimber.getEncoder().setPosition(0.0); } catch (Exception ignored) {}
        try { rightClimber.getEncoder().setPosition(0.0); } catch (Exception ignored) {}
    }

    // Emergency stop
    public void eStop() {
        setVoltage(0.0);
    }

    public void stop() {
        leftClimber.setVoltage(0);
        rightClimber.setVoltage(0);
    }

    public boolean isLeftHallSensorTriggered() {
        leftHallSensorTriggered = !leftHallSensor.get(); // active low
        return leftHallSensorTriggered;
    }

    public boolean isRightHallSensorTriggered() {
        rightHallSensorTriggered = !rightHallSensor.get(); // active low
        return rightHallSensorTriggered;
    }

    public boolean isStalled() {
        return m_debouncer.calculate(getCurrent() > 20.0);
    }

    // Precoded motion commands (safe)
    public void toTop() {
        leftClimber.set(0.5);
        rightClimber.set(0.5);
    }

    public void toBottom() {
        leftClimber.set(-0.5);
        rightClimber.set(-0.5);
    }
}