// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Indexer.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Intake.StoreNote.StoreNoteSequence;
//import frc.robot.commands.LEDs.RainbowLEDs;
import frc.robot.commands.LEDs.SignalStoreNote;
import frc.robot.commands.Shooter.*;
import frc.robot.subsystems.*;
import frc.robot.testing.routines.DrivetrainTest;
import frc.robot.testing.routines.FlywheelTest;
import frc.robot.testing.routines.IntakeTest;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final PS4Controller driverController = Button.controller1;
  private final GenericHID operatorController = Button.controller2;

  // ==========================
  // Subsystems
  // ==========================

  public final Drivetrain m_drivetrain = new Drivetrain();
  public final Shooter m_shooter = new Shooter();
  public final Intake m_intake = new Intake();
  public final Indexer m_indexer = new Indexer();
  public final ShooterLimelight m_shooterLimelight = new ShooterLimelight();
  public final IntakeLimelight m_intakeLimelight = new IntakeLimelight();
  public final LEDs m_leds = new LEDs();

  public final AutoSelector m_autoSelector = new AutoSelector(m_drivetrain, m_shooter, m_intake, m_indexer);
  public final ShuffleboardData m_shuffleboardData = new ShuffleboardData(m_drivetrain, m_shooter,
      m_intake/* , m_indexer */, m_autoSelector);

  // ==========================
  // Commands
  // ==========================

  /* Drivetrain */
  public final SwerveDriveCommand m_swerveDriveOpenLoop = new SwerveDriveCommand(m_drivetrain, driverController, true);
  public final SwerveDriveCommand m_swerveDriveClosedLoop = new SwerveDriveCommand(m_drivetrain, driverController,
      false);
  public final SetPercentOutputCommand m_setDrivePercentOutput = new SetPercentOutputCommand(m_drivetrain, 0.10, 0);
  public final ResetFieldOrientedHeading m_resetFieldOrientedHeading = new ResetFieldOrientedHeading(m_drivetrain);
  // public final Command m_pathfindToSource = AutoBuilder
  //     .pathfindToPose(new Pose2d(1.32, 1.32, Rotation2d.fromDegrees(-120)), Constants.kDrivetrain.PATH_CONSTRAINTS);
  public final Command m_pathfindToAmp = AutoBuilder.pathfindToPose(new Pose2d(1.84, 5.67, Rotation2d.fromDegrees(90)),
       Constants.kDrivetrain.PATH_CONSTRAINTS);
  public final Command m_sysIDDriveRoutine = new ProxyCommand(m_drivetrain::getSysIDDriveRoutine);
  public final AlignWithSpeaker2Command m_alignWithSpeaker = new AlignWithSpeaker2Command(m_drivetrain, driverController, false, true);
  /*public final SequentialCommandGroup m_alignWithSpeaker = new SequentialCommandGroup(
      new AlignWithSpeaker1Command(m_drivetrain, driverController, false, true),
      new AlignWithSpeaker2Command(m_drivetrain, driverController, false, true));*/

  /* Shooter */
  // public final PrepareShooterCommand m_prepareShooter = new PrepareShooterCommand(m_shooter, m_drivetrain);
  public final ManualShooterCommand m_manualShooter = new ManualShooterCommand(m_shooter, m_drivetrain, m_indexer, operatorController);
  public final ResetAlternateAngleCommand m_resetAlternateAngle = new ResetAlternateAngleCommand(m_shooter);
  public final StowShooterCommand m_stow = new StowShooterCommand(m_shooter);
  public final ToAmpPositionCommand m_toAmpAngle = new ToAmpPositionCommand(m_shooter, operatorController);
  public final AlignWithAmpCommand m_alignAmp = new AlignWithAmpCommand(m_drivetrain, driverController, false, true);
  public final ToClimbPositionCommand m_ToClimbPositionCommand = new ToClimbPositionCommand(m_shooter);
  public final ShootCommand m_shoot = new ShootCommand(m_shooter, m_indexer, m_drivetrain, driverController);
  public final ClimbLockCommand m_lockClimber = new ClimbLockCommand(m_shooter, operatorController);
  public final SubwooferShotCommand m_subwooferShotCommand = new SubwooferShotCommand(m_shooter, m_indexer, m_drivetrain, driverController);
  public final PassNoteCommand m_pass = new PassNoteCommand(m_shooter, m_indexer);

  /* Intake */
  public final RunIntakeCommand m_runIntakeIn = new RunIntakeCommand(m_intake, 0.5);
  public final RunIntakeCommand m_runIntakeOut = new RunIntakeCommand(m_intake, -0.5);
  public final StoreNoteSequence m_storeNote = new StoreNoteSequence(m_indexer, m_intake);
  public final ReverseWhileNoteStoredCommand m_reverseWhileNoteStored = new ReverseWhileNoteStoredCommand(m_intake, m_indexer, operatorController);

  /* Indexer */
  // public final RunIndexerCommand m_runIndexerUp = new
  // RunIndexerCommand(m_indexer, 0.3); // Manual Stow
  // public final StoreNote m_runIndexerUp = new StoreNote(m_indexer, m_intake); // Auto stow
  // public final RunIndexerCommand m_runIndexerDown = new RunIndexerCommand(m_indexer, -0.3);
  public final ManualIndexerCommand m_manualIndexer = new ManualIndexerCommand(m_indexer, operatorController);
  // public final NudgeIndexer m_nudgeIndexer = new NudgeIndexer(m_indexer);
  public final AlignWithNoteCommand m_alignNote = new AlignWithNoteCommand(m_drivetrain, m_indexer);

  public final RecordFFDataCommand m_ffData = new RecordFFDataCommand(m_shooter);

  /* LEDs */
  public final SignalStoreNote m_signalStoreNote = new SignalStoreNote(m_leds, m_indexer, m_intake);

  /* Tests */
  public final FlywheelTest m_flywheelTest = new FlywheelTest(m_shooter);
  public final DrivetrainTest m_DrivetrainTest = new DrivetrainTest(m_drivetrain);
  public final IntakeTest m_IntakeTest = new IntakeTest(m_intake);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureDriveBindings();

    m_drivetrain.setDefaultCommand(m_swerveDriveClosedLoop);
    m_indexer.setDefaultCommand(m_manualIndexer);
    m_shooter.setDefaultCommand(m_manualShooter);
    m_intake.setDefaultCommand(m_reverseWhileNoteStored);
    m_leds.setDefaultCommand(m_signalStoreNote);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDebugBindings() {

    // /* Drivetrain Bindings */
    // Button.square.whileTrue(m_setDrivePercentOutput);
    // Button.triangle.onTrue(m_resetFieldOrientedHeading);
    // Button.controlPadRight1.whileTrue(m_pathfindToSource);
    // Button.controlPadLeft1.whileTrue(m_pathfindToAmp);
    // Button.rightBumper1.and(Button.triangle).whileTrue(m_driveQuasistaicForward);
    // Button.rightBumper1.and(Button.square).whileTrue(m_driveQuasistaicReverse);
    // Button.rightBumper1.and(Button.cross).whileTrue(m_driveDynamicForward);
    // Button.rightBumper1.and(Button.circle).whileTrue(m_driveDynamicReverse);
    // Button.leftBumper1.onTrue(m_resetToAprilTagPose);
    // Button.circle.whileTrue(m_alignWithSpeaker);

    // /* Shooter Bindings */
    // Button.x.toggleOnTrue(m_prepareShooter);
    // Button.leftBumper2.onTrue(m_resetAlternateAngle);
    // Button.rightBumper2.toggleOnTrue(m_toAmpAngle);
    // Button.leftTrigger2.toggleOnTrue(m_nudgeIndexer);
    // Button.rightTrigger2.onTrue(m_ToClimbPositionCommand);

    // /* Intake Bindings */
    // Button.b.toggleOnTrue(m_runIntakeIn);
    // Button.a.whileTrue(m_runIntakeOut);

    // /* Indexer Bindings */
    // Button.y.whileTrue(m_runIndexerUp);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDriveBindings() {

    // ================
    // Driver Controls
    // ================

    /* Drivetrain */
    Button.rightTrigger1.whileTrue(m_shoot);
    Button.leftTrigger1.or(Button.rightBumper2).onTrue(m_stow);
    Button.cross1.toggleOnTrue(m_toAmpAngle);
    Button.triangle1.onTrue(m_resetFieldOrientedHeading);
    //Button.square1.whileTrue(m_alignAmp);
    Button.leftBumper1.whileTrue(m_alignNote);
    Button.controlPadLeft1.toggleOnTrue(m_sysIDDriveRoutine);

    /* Intake */
    Button.controlPadUp1.onTrue(new InstantCommand(() -> Constants.kIntake.INTAKE_SPEED += 0.1));
    Button.controlPadDown1.onTrue(new InstantCommand(() -> Constants.kIntake.INTAKE_SPEED -= 0.1));
    Button.controlPadRight1.onTrue(m_ffData);
    Button.leftBumper1.or(Button.cross2).onTrue(m_storeNote);

    // ==================
    // Operator Controls
    // ==================

    Button.leftTrigger2.onTrue(m_lockClimber);
    Button.rightTrigger2.onTrue(m_ToClimbPositionCommand);
    Button.triangle2.onTrue(m_runIntakeOut);
    Button.square2.onTrue(m_resetAlternateAngle);
    Button.leftStickClick2.onTrue(m_ffData);
    Button.rightBumper2.whileTrue(m_subwooferShotCommand);
    Button.leftBumper2.onTrue(m_pass);

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
