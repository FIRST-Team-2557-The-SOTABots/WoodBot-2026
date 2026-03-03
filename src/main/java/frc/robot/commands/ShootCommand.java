// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterHood;
import frc.robot.Constants;
public class ShootCommand extends Command {

    private final DriveSubsystem drive;
    private final ShooterHood hood;

    public ShootCommand(DriveSubsystem drive, ShooterHood hood) {
        this.drive = drive;
        this.hood = hood;

        addRequirements(hood);
    }

    @Override
    public void execute() {

        Pose2d pose = drive.getPose();

        double distance =
            pose.getTranslation().getDistance(Constants.GOAL);

        double hoodAngle =
            hood.getAngleFromDistance(distance);

        hood.setAngle(hoodAngle);
    }
}
