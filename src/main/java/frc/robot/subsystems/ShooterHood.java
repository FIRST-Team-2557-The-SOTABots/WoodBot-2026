// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {

  private Servo shooterHoodServoRight;
  private Servo shooterHoodServoLeft;

  private static final double MIN_ANGLE = 22;  // temporary guess
  private static final double MAX_ANGLE = 80;  // temporary guess

  /** Creates a new ShooterHood. */
  private final InterpolatingDoubleTreeMap hoodMap =
    new InterpolatingDoubleTreeMap();

public ShooterHood() {

    shooterHoodServoRight = new Servo(
        Constants.ShooterConstants.kShooterHoodServoRightChannel);

    shooterHoodServoLeft = new Servo(
        Constants.ShooterConstants.kShooterHoodServoLeftChannel);

    // TEST THIS NOWW
    hoodMap.put(2.0, 28.0);
    hoodMap.put(3.0, 35.0);
    hoodMap.put(4.0, 42.0);
}

  public void setAngle(double angleDegrees) {

    // Clamp to safe range
    angleDegrees = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDegrees));

    // Convert angle -> 0 to 1 for servo
    double servoValue =
        (angleDegrees - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);

    shooterHoodServoRight.set(servoValue);
    shooterHoodServoLeft.set(servoValue);
  }

  public double getAngleFromDistance(double distance) {
    double angle = hoodMap.get(distance);
    return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
