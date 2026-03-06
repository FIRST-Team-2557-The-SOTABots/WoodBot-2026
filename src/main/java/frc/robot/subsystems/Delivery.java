package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Delivery extends SubsystemBase {

  public static SparkFlex delivery;
  public static SparkFlexConfig deliveryConfig;

  private int stutterCounter = 0;

  public Delivery() {
    delivery = new SparkFlex(
      Constants.DeliveryConstants.kDeliveryCanId,
      MotorType.kBrushless);

    deliveryConfig = Configs.DeliveryConfigs.deliveryConfig;

    delivery.configure(
      deliveryConfig,
      com.revrobotics.ResetMode.kNoResetSafeParameters,
      com.revrobotics.PersistMode.kPersistParameters);
  }

  public void setDeliveryVoltage(double voltage) {
    delivery.setVoltage(voltage);
  }

  public void stutter(double voltage) {
    stutterCounter++;

    if (stutterCounter % 24 < 12) {
      delivery.setVoltage(voltage);
    } else {
      delivery.setVoltage(0);
    }
  }

  public void resetStutterStop() {
    stutterCounter = 0;
    delivery.setVoltage(0);
  }

  @Override
  public void periodic() {}
}