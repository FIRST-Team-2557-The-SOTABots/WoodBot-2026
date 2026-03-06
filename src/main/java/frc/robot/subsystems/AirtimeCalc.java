package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

public class AirtimeCalc extends SubsystemBase {

    private final DriveSubsystem m_drive;
    private final double flywheelRadiusMeters = 0.1016; // 4-inch diameter = 0.1016 m radius
    private double currentAirtime = 0;

    /**
     * Constructor
     * @param drive Pass in your robot's DriveSubsystem
     */
    public AirtimeCalc(DriveSubsystem drive) {
        m_drive = drive;
    }

    /**
     * Runs every 20ms automatically. Computes airtime and stores it.
     */
    @Override
    public void periodic() {

        if (Shooter.shooterFlyWheelLeft == null) return;

        // Get flywheel RPM
        double rpm = Shooter.shooterFlyWheelLeft.getEncoder().getVelocity();

        // Convert RPM -> linear velocity (m/s)
        double velocity = (rpm * 2 * Math.PI * flywheelRadiusMeters) / 60.0;

        // Apply slip factor (70-85% actual wheel exit velocity)
        velocity *= 0.8;

        // Get current robot position
        Pose2d pose = m_drive.getPose();

        // Calculate distance to goal
        double distance = pose.getTranslation().getDistance(Constants.GOAL);

        // Compute airtime safely
        if (velocity > 0) {
            currentAirtime = distance / velocity;
        } else {
            currentAirtime = 0;
        }

        // Optional: display live on SmartDashboard
        SmartDashboard.putNumber("Shooter Airtime", currentAirtime);
    }

    /**
     * Returns the latest computed airtime in seconds
     */
    public double getCurrentAirtime() {
        return currentAirtime;
    }
}