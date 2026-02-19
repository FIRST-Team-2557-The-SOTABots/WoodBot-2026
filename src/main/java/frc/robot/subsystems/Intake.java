// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkFlex intake;
  private SparkMax intakeROT;

  private SparkAbsoluteEncoder intakeEncoder;

  private SparkClosedLoopController intakePID;

  private SparkFlexConfig intakeConfig;
  private SparkMaxConfig intakeRotConfig;

  private double intakePosition;

  public Intake() {
    intake = new SparkFlex(
      Constants.IntakeConstants.kIntakeIntakeCanId, 
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    intakeROT = new SparkMax(
      Constants.IntakeConstants.kIntakeROTCanId, 
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    

    intakePID = intakeROT.getClosedLoopController();

    intakeEncoder = intakeROT.getAbsoluteEncoder();


    intakeConfig = Configs.IntakeConfigs.intakeConfig;
    intakeRotConfig = Configs.IntakeConfigs.intakeROTConfig;


    intake.configure(
      intakeConfig,
     ResetMode.kNoResetSafeParameters,
     PersistMode.kPersistParameters);

    intakeROT.configure(
      intakeRotConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  public void setIntakeVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  public double getIntakePosition() {
    return intakeEncoder.getPosition();
  }

  public void setIntakePosition(double position) {
    this.intakePosition = position;
  }

  @Override
  public void periodic() {
    intakePID.setSetpoint(intakePosition, ControlType.kPosition);
  }
}
