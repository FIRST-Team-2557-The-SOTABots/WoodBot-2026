// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private SparkFlex shooterFlyWheelLeft;
  private SparkFlex shooterFlyWheelMiddle;
  private SparkFlex shooterFlyWheelRight;
  private SparkFlex shooterDelivery;

  private SparkFlexConfig shooterFlyWheelLeftConfig = new SparkFlexConfig();
  private SparkFlexConfig shooterFlyWheelMiddleConfig = new SparkFlexConfig();
  private SparkFlexConfig shooterFlyWheelRightConfig = new SparkFlexConfig();
  private SparkFlexConfig shooterDeliveryConfig = new SparkFlexConfig();

  /** Creates a new ShooterFlyWheels. */
  public Shooter() {
    shooterFlyWheelLeft = new SparkFlex(
      Constants.ShooterConstants.kShooterFlyWheelLeftCanId,
      MotorType.kBrushless);

    shooterFlyWheelMiddle = new SparkFlex(
      Constants.ShooterConstants.kShooterFlyWheelMiddleCanId, 
      MotorType.kBrushless);

    shooterFlyWheelRight = new SparkFlex(
      Constants.ShooterConstants.kShooterFlyWheelRightCanId,
     MotorType.kBrushless);
     
    shooterDelivery = new SparkFlex(
      Constants.ShooterConstants.kShooterDeliveryCanId, 
      MotorType.kBrushless);


    shooterFlyWheelLeft.configure(
      shooterFlyWheelLeftConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);

    shooterFlyWheelMiddle.configure(
      shooterFlyWheelMiddleConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);

    shooterFlyWheelRight.configure(
      shooterFlyWheelRightConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);

    shooterDelivery.configure(
      shooterDeliveryConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
