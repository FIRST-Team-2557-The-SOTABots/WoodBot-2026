// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Climber m_climber = new Climber();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    // For convenience a programmer could change this when going to competition.
    boolean isCompetition = true;

    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith(""))
        : stream
    );

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();


    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    m_driverController.start().onTrue(Commands.runOnce(
      () -> m_robotDrive.zeroHeading()
      , m_robotDrive));

    
    // Climbs while buttons are being held and stops once it ends
    m_driverController.rightBumper().and(m_driverController.leftBumper())
    .whileTrue(Commands.startEnd(
        m_climber::climb, 
        () -> { 
            m_climber.stop(); 
            m_climber.stopClimb(); 
        }, 
        m_climber));
    
    // Brings climber to 30 inches using hall effect sensors (Declimb or Climb preperation)
    m_driverController.rightTrigger().and(m_driverController.leftTrigger())
      .whileTrue(
        Commands
        .startEnd(
          m_climber::toTop, 
          m_climber::stop, 
          m_climber)
      .until(
        () -> m_climber.isLeftHallSensorTriggered() || m_climber.isRightHallSensorTriggered()));

    // Back Button: Auto-retract until the motor stalls at the bottom
    m_driverController.back().onTrue(
      Commands.startEnd(
        m_climber::toBottom, // Starts moving down at -0.5
        m_climber::stop,     // Stops when done
        m_climber            // Requirement
      ).until(m_climber::isStalled) // Stops as soon as the 20A spike is detected
    );

      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
