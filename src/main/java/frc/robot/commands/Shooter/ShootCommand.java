// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.AlignWithSpeakerCommand;
import frc.robot.commands.Indexer.NudgeIndexer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/**
 * Prepares the shooter for shooting, aligns the robot with the speaker (while maintaining driver control of translation) and then fires when everything is ready
 */
public class ShootCommand extends ParallelDeadlineGroup {
  /** Creates a new ShootCommand. */
  public ShootCommand(Shooter shooter, Indexer indexer, Drivetrain drivetrain, GenericHID driveController) {
    super(new SequentialCommandGroup(new WaitUntilCommand(() -> ready(shooter, indexer, drivetrain)), new NudgeIndexer(indexer)));
    PrepareShooterCommand prepareShooter = new PrepareShooterCommand(shooter, indexer, drivetrain);
    AlignWithSpeakerCommand alignWithSpeakerCommand = new AlignWithSpeakerCommand(drivetrain, driveController, true, true);
    addCommands(prepareShooter, alignWithSpeakerCommand);
  }

  private static boolean ready(Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
    // Check if the flywheels are spinning fast enough
    boolean atSpeed = shooter.detectFlywheelSpeed(Constants.kShooter.FLYWHEEL_RPM_ACCEPTABLE_ERROR);
    // Check if the shooter is aimed vertically accurately enough
    boolean verticallyAimed = shooter.detectShooterAngle(Constants.kShooter.VERTICAL_AIM_ACCEPTABLE_ERROR);
    // find the angle to speaker
    Translation2d currentPosition = drivetrain.getPose().getTranslation();
    Translation2d directionToSpeaker = Constants.kFieldPositions.SPEAKER_POSITION.minus(currentPosition);
    Rotation2d targetAngle = directionToSpeaker.getAngle();
    // Find the error in the drivetrain angle
    double drivetrainAngleError = targetAngle.minus(drivetrain.getRotation2d()).getDegrees();
    // Check if the robot is aimed horizontally accurately enough
    boolean horizontallyAimed = Math.abs(drivetrainAngleError) < Constants.kShooter.HORIZONTAL_AIM_ACCEPTABLE_ERROR;
    // Find the drivetrain speed
    ChassisSpeeds chassisSpeeds = drivetrain.getChassisSpeeds();
    double speed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    // Check if the robot is moving slow enough to shoot
    boolean stopped = speed < Constants.kShooter.MAXIMUM_SHOOTING_DRIVETRAIN_SPEED;

    return atSpeed && verticallyAimed && horizontallyAimed && stopped;
  }
}
