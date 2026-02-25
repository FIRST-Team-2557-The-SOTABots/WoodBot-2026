// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Configs;
import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterHood;

public class Shooter extends SubsystemBase {

  private SparkFlex shooterFlyWheelLeft;
  private SparkFlex shooterFlyWheelMiddle;
  private SparkFlex shooterFlyWheelRight;


  private SparkFlexConfig shooterFlyWheelLeftConfig;
  private SparkFlexConfig shooterFlyWheelMiddleConfig;
  private SparkFlexConfig shooterFlyWheelRightConfig;

  private PIDController shooterFlyWheelPIDController;

  private DriveSubsystem kDrive;

  private ShooterHood kShooterHood;

  /** Creates a new ShooterFlyWheels. */
  public Shooter(DriveSubsystem kDrive, ShooterHood kShooterHood) {
    this.kDrive = kDrive;
    this.kShooterHood = kShooterHood;


    shooterFlyWheelLeft = new SparkFlex(
      Constants.ShooterConstants.kShooterFlyWheelLeftCanId,
      MotorType.kBrushless);

    shooterFlyWheelMiddle = new SparkFlex(
      Constants.ShooterConstants.kShooterFlyWheelMiddleCanId, 
      MotorType.kBrushless);

    shooterFlyWheelRight = new SparkFlex(
      Constants.ShooterConstants.kShooterFlyWheelRightCanId,
     MotorType.kBrushless);


    shooterFlyWheelLeftConfig = Configs.ShooterConfigs.ShooterFlyWheelLeftConfig;
    shooterFlyWheelMiddleConfig = Configs.ShooterConfigs.ShooterFlyWheelMiddleConfig;
    shooterFlyWheelRightConfig = Configs.ShooterConfigs.ShooterFlyWheelRightConfig;



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


  }

  // Set the voltage of the flywheel motors to control the speed of the flywheels
  public void setFlyWheelVoltage(double voltage) {
    shooterFlyWheelLeft.setVoltage(voltage);
    shooterFlyWheelMiddle.setVoltage(voltage);
    shooterFlyWheelRight.setVoltage(voltage);
  }

  // public void setFlyWheelRPM(double rpm) {
  //   shooterFlyWheelLeft.getClosedLoopController().setReference(rpm, );
  //   shooterFlyWheelMiddle.getClosedLoopController().setReference(rpm, com.revrobotics.ControlType.kVelocity);
  //   shooterFlyWheelRight.getClosedLoopController().setReference(rpm, com.revrobotics.ControlType.kVelocity);
  // }

  // Set the voltage of the delivery motor to control the speed of the delivery mechanism


  //average the velocity of the three flywheels to get a more accurate reading of the flywheel speed
  public double getFlyWheelVelocity() {
    return (shooterFlyWheelLeft.getEncoder().getVelocity() +
           shooterFlyWheelMiddle.getEncoder().getVelocity() +
           shooterFlyWheelRight.getEncoder().getVelocity()) / 3.0;
  }

  public void shootStationary(double x, double y) {
    Translation2d diff =
        new Translation2d(x, y).minus(kDrive.getPose().getTranslation());

    double distance = diff.getNorm();

  }
  
  // Translation2d hub = Constants.FieldConstants.kHubPosition;

  // double hoodAngle = getDesiredHoodAngle(hub);
  // double flywheelRPM = getDesiredFlywheelRPM(hub);

  // {kShooterHood.setHoodAngle(hoodAngle);
  // setFlywheelRPM(flywheelRPM);}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
