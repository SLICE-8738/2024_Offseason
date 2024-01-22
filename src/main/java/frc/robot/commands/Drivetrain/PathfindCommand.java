// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class PathfindCommand extends Command {

  private final Drivetrain m_drivetrain;
  private final Pose2d targetPose;
  private Command pathfindCommand;

  /** Creates a new PathfindCommand. */
  public PathfindCommand(Drivetrain drivetrain, Pose2d targetPose) {

    m_drivetrain = drivetrain;
    this.targetPose = targetPose;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    pathfindCommand = AutoBuilder.pathfindToPose(targetPose, Constants.kAutonomous.kPathConstraints);
    pathfindCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    pathfindCommand.cancel();
    m_drivetrain.stopDrive();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }
  
}
