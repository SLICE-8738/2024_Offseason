// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AlignToPoseCommand extends Command {
  /** Creates a new AlignToPoseCommand. */
  private final Drivetrain m_drivetrain;
  private final Pose2d targetPose;
  private final double driveDistance;
  private boolean forceStop;

  private PIDController xController, yController, thetaController;

  public AlignToPoseCommand(Drivetrain drivetrain, Pose2d targetPose, double driveDistance) {

    m_drivetrain = drivetrain;
    this.targetPose = targetPose;
    this.driveDistance = driveDistance;

    xController = new PIDController(0.1, 0, 0);
    yController = new PIDController(0.1, 0, 0);
    thetaController = new PIDController(Constants.kDrivetrain.kPSpeakerAlignRotation, Constants.kDrivetrain.kISpeakerAlignRotation, Constants.kDrivetrain.kDSpeakerAlignRotation);

    forceStop = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation()) > driveDistance) {
      forceStop = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = m_drivetrain.getPose();

    double translationX = xController.calculate(robotPose.getX(), targetPose.getX());
    double translationY = yController.calculate(robotPose.getY(), targetPose.getY());
    double turnAmount = thetaController.calculate(robotPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());

    m_drivetrain.drive(
        new Transform2d(new Translation2d(translationX, translationY), Rotation2d.fromDegrees(-turnAmount)),
        false,
        true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return forceStop;
  }
}
