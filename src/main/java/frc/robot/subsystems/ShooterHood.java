// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {

  private Servo shooterHoodServoRight;
  private Servo shooterHoodServoLeft;

  /** Creates a new ShooterHood. */
  public ShooterHood() {
    shooterHoodServoRight = new Servo(
      Constants.ShooterConstants.kShooterHoodServoRightChannel);

    shooterHoodServoLeft = new Servo(
      Constants.ShooterConstants.kShooterHoodServoLeftChannel);
  }

  public void setHoodAngle(double length) {
    shooterHoodServoRight.setAngle(length);
    shooterHoodServoLeft.setAngle(length);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
