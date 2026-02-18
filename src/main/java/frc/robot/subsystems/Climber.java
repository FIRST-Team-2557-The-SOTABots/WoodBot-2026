package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Configs;

public class Climber extends SubsystemBase {
  private SparkFlex leftClimber;
  private SparkFlex rightClimber;

  private SparkFlexConfig leftClimberConfig;
  private SparkFlexConfig rightClimberConfig;
  /** Creates a new Climber. */
  public Climber() {
    leftClimber = new SparkFlex(Constants.ClimberConstants.kClimberLeftCanId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    rightClimber = new SparkFlex(Constants.ClimberConstants.kClimberRightCanId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    leftClimber.configure(leftClimberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightClimber.configure(rightClimberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    leftClimberConfig = Configs.ClimberConfigs.leftClimberConfig;
    rightClimberConfig = Configs.ClimberConfigs.rightClimberConfig;
  }

  @Override
  public void periodic() {
      // This method will be called once per scheduler run
  }

  public void setVoltage(double voltage) {
    leftClimber.setVoltage(voltage);
    rightClimber.setVoltage(voltage);
  }
}
