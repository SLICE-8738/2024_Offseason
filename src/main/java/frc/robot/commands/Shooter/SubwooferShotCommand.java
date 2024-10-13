// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Constants;
import frc.robot.commands.indexer.NudgeIndexer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Prepares the shooter for shooting, aligns the robot with the speaker (while maintaining driver control of translation) and then fires when everything is ready
 */
public class SubwooferShotCommand extends ParallelDeadlineGroup {

  /** Creates a new SubwooferShotCommand for teleop. */
  public SubwooferShotCommand(Shooter shooter, Indexer indexer, Drivetrain drivetrain, GenericHID driveController) {
    super(
      new SequentialCommandGroup(
        new WaitCommand(0.1), 
        new WaitUntilCommand(() -> ready(shooter, indexer, drivetrain)), 
        new NudgeIndexer(indexer)),
      new PrepareShooterCommand(shooter, drivetrain, true));
  }

  /** Creates a new SubwooferShotCommand for autonomous. */
  public SubwooferShotCommand(Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
    super(
      new SequentialCommandGroup(
        new WaitCommand(0.1),
        new ParallelRaceGroup(new WaitCommand(2.5), new WaitUntilCommand(() -> ready(shooter, indexer, drivetrain))),
        new NudgeIndexer(indexer)),
      new PrepareShooterCommand(shooter, drivetrain, true)
    );
  }

  private static boolean ready(Shooter shooter, Indexer indexer, Drivetrain drivetrain) {

    // Check if the flywheels are spinning fast enough
    boolean atSpeed = shooter.atTargetSpeed(Constants.kShooter.FLYWHEEL_RPM_ACCEPTABLE_ERROR);
    // Find the drivetrain speed
    ChassisSpeeds chassisSpeeds = drivetrain.getChassisSpeeds();
    // Check if the robot is moving slow enough to shoot
    boolean stopped = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)< Constants.kShooter.MAXIMUM_SHOOTING_DRIVETRAIN_SPEED;

    return atSpeed && stopped;
  }
}
