package frc.robot.subsystems;

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

        positionPid = new PIDController(
                Constants.ClimberConstants.kClimberP,
                0, // I = 0 for climber
                Constants.ClimberConstants.kClimberD
        );
        positionPid.setTolerance(Constants.ClimberConstants.kClimberPositionTolerance);
    }

    @Override
    public void periodic() {
        isLeftHallSensorTriggered();
        isRightHallSensorTriggered();

        // if (!positionControlEnabled) return;

        // double leftPos, rightPos;
        // try {
        //     leftPos = leftClimber.getEncoder().getPosition();
        //     rightPos = rightClimber.getEncoder().getPosition();
        // } catch (Exception ex) {
        //     positionControlEnabled = false;
        //     return;
        // }

        // double currentPos = (leftPos + rightPos) / 2.0;
        // double output = positionPid.calculate(currentPos, targetPosition);

        // // Limit max voltage (reduce overshoot)
        // double maxV = Constants.ClimberConstants.kClimberMaxVoltage;
        // output = Math.max(-maxV, Math.min(maxV, output));

        // // Current readings
        // double leftCurrent = leftClimber.getOutputCurrent();
        // double rightCurrent = rightClimber.getOutputCurrent();
        // double bottomThreshold = Constants.ClimberConstants.kClimberBottomDetectCurrent;

        // // Independent side clamping
        // double leftOutput = output;
        // double rightOutput = output;

        // // Clamp downward motion if hitting bottom
        // if (leftOutput < 0 && (leftHallSensorTriggered || leftCurrent > bottomThreshold)) leftOutput = 0;
        // if (rightOutput < 0 && (rightHallSensorTriggered || rightCurrent > bottomThreshold)) rightOutput = 0;

        // // Clamp upward motion if hitting top Hall sensor
        // if (leftOutput > 0 && leftHallSensorTriggered) leftOutput = 0;
        // if (rightOutput > 0 && rightHallSensorTriggered) rightOutput = 0;

        // // Reset integrator if fully clamped
        // if (leftOutput == 0 && rightOutput == 0) positionPid.reset();

        // // Apply voltage
        // leftClimber.setVoltage(leftOutput);
        // rightClimber.setVoltage(rightOutput);
    }

    // Manual voltage control
    public void setVoltage(double voltage) {
        leftClimber.setVoltage(voltage);
        rightClimber.setVoltage(voltage);
    }

    // PID control
    public void setTargetPositionRotations(double position) {
        targetPosition = position;
        positionPid.reset();
        positionControlEnabled = true;
    }

    public void disablePositionControl() { positionControlEnabled = false; }

    public void enablePositionControl() {
        positionPid.reset();
        positionControlEnabled = true;
    }

    public boolean atSetpoint() { return positionPid.atSetpoint(); }

    // Encoder (never re-zeroed in match)
    public void zeroEncoders() {
        try { leftClimber.getEncoder().setPosition(0.0); } catch (Exception ignored) {}
        try { rightClimber.getEncoder().setPosition(0.0); } catch (Exception ignored) {}
    }

    // Emergency stop
    public void eStop() {
        setVoltage(0.0);
        disablePositionControl();
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

    // Precoded motion commands (safe)
    public void toTop() {
        leftClimber.set(0.5);
        rightClimber.set(0.5);
    }

    public void toBottom() {
        setTargetPositionRotations(-100.0); // small relative value
    }
}