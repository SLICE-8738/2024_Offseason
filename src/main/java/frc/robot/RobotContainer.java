// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Drivetrain.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final PS4Controller driverController = Button.controller1;

  // ==========================
  // Subsystems
  // ==========================

  public final Drivetrain m_drivetrain = new Drivetrain();
  public final Limelights m_limelights = new Limelights();

  public final AutoSelector m_autoSelector = new AutoSelector(m_drivetrain);
  public final ShuffleboardData m_shuffleboardData = new ShuffleboardData(m_drivetrain, m_autoSelector);

  // ==========================
  // Commands
  // ==========================

  /* Drivetrain */
  public final SwerveDriveCommand m_swerveDriveOpenLoop = new SwerveDriveCommand(m_drivetrain, driverController, true, true);
  public final SwerveDriveCommand m_swerveDriveClosedLoop = new SwerveDriveCommand(m_drivetrain, driverController, false, true);
  public final SetPercentOutputCommand m_setDrivePercentOutput = new SetPercentOutputCommand(m_drivetrain, 0.1, 0);
  public final ResetFieldOrientedHeading m_resetFieldOrientedHeading = new ResetFieldOrientedHeading(m_drivetrain);
  public final Command m_pathfindToSource = AutoBuilder.pathfindToPose(new Pose2d(1.32, 1.32, Rotation2d.fromDegrees(-120)), Constants.kDrivetrain.PATH_CONSTRAINTS);
  public final Command m_pathfindToAmp = AutoBuilder.pathfindToPose(new Pose2d(1.84, 7.67, Rotation2d.fromDegrees(90)), Constants.kDrivetrain.PATH_CONSTRAINTS);
  public final Command m_driveQuasistaicForward = m_drivetrain.getSysIdDriveQuasistatic(Direction.kForward);
  public final Command m_driveQuasistaicReverse = m_drivetrain.getSysIdDriveQuasistatic(Direction.kReverse);
  public final Command m_driveDynamicForward = m_drivetrain.getSysIdDriveDynamic(Direction.kForward);
  public final Command m_driveDynamicReverse = m_drivetrain.getSysIdDriveDynamic(Direction.kReverse);
  //public final ConditionalCommand m_limelightAlign = new ConditionalCommand(m_aprilTagAlign, m_noteAlign, noteDetected);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    m_drivetrain.setDefaultCommand(m_swerveDriveClosedLoop);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    /* Drivetrain Bindings */
    Button.square.whileTrue(m_setDrivePercentOutput);
    Button.triangle.onTrue(m_resetFieldOrientedHeading);
    Button.controlPadRight1.whileTrue(m_pathfindToSource);
    Button.pathfindToAmp.whileTrue(m_pathfindToAmp);

    Button.rightBumper1.and(Button.triangle).onTrue(m_driveQuasistaicForward);
    Button.rightBumper1.and(Button.square).onTrue(m_driveQuasistaicReverse);
    Button.rightBumper1.and(Button.cross).onTrue(m_driveDynamicForward);
    Button.rightBumper1.and(Button.circle).onTrue(m_driveDynamicReverse);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_autoSelector.getAutoRoutine();
    
  }

}
