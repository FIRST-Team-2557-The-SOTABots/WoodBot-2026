package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldUtil {
    private static final Translation2d RED_GOAL = new Translation2d(13.52, 4.13);
    private static final Translation2d BLUE_GOAL = new Translation2d(4.02, 4.13);

    public static Translation2d getGoalForAlliance() {
        // Ask the driver station which alliance we're on
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red); // default Red if unknown

        if (alliance == Alliance.Red) {
            return RED_GOAL;
        } else {
            return BLUE_GOAL;
        }
    }
}