package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldUtil {
    private static final double FIELD_WIDTH = 8.23; // meters
    private static final Translation2d RED_GOAL = new Translation2d(8.23, 2.64);

    public static Translation2d getGoalForAlliance() {
        // Ask the driver station which alliance we're on
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red); // default Red if unknown

        if (alliance == Alliance.Red) {
            return RED_GOAL;
        } else {
            // Mirror the Y coordinate for blue alliance
            return new Translation2d(RED_GOAL.getX(), FIELD_WIDTH - RED_GOAL.getY());
        }
    }
}