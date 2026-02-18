// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkFlex intake;
  private SparkFlex intakeROT;

  private SparkFlexConfig intakeConfig;
  private SparkFlexConfig intakeRotConfig;

  public Intake() {
    intake = new SparkFlex(
      Constants.IntakeConstants.kIntakeIntakeCanId, 
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    intakeROT = new SparkFlex(
      Constants.IntakeConstants.kIntakeROTCanId, 
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);


    intake.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    intakeROT.configure(intakeRotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
