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
  // temporary guess

  /** Creates a new ShooterHood. */
  private final InterpolatingDoubleTreeMap hoodMap =
    new InterpolatingDoubleTreeMap();

public ShooterHood() {

    shooterHoodServoRight = new Servo(
        Constants.ShooterConstants.kShooterHoodServoRightChannel);

    shooterHoodServoLeft = new Servo(
        Constants.ShooterConstants.kShooterHoodServoLeftChannel);

    // TEST THIS NOWW
    hoodMap.put(2.0, 0.26);
    hoodMap.put(3.0, 0.34);
    hoodMap.put(4.0, 0.42);
}

  public void setAngle(double angleDegrees) {

    // Clamp to safe range

    shooterHoodServoRight.set(angleDegrees);
    shooterHoodServoLeft.set(angleDegrees);
  }

  public double getAngleFromDistance(double distance) {
    double pos = hoodMap.get(distance);
    return pos;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
