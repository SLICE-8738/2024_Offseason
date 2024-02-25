// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShooterMath;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class SpinFlywheelCommand extends Command {
  private final Shooter m_shooter;
  private final Drivetrain m_drivetrain;

  /** Creates a new SpinFlywheelCommand. */
  public SpinFlywheelCommand(Shooter shooter, Drivetrain drivetrain) {
    m_shooter = shooter;
    m_drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d currentPosition = m_drivetrain.getPose().getTranslation();
    double distanceToSpeaker = currentPosition.getDistance(Constants.kFieldPositions.SPEAKER_POSITION);
    double flywheelVelocity = ShooterMath.getShot(distanceToSpeaker).getFlywheelVelocity();
    m_shooter.spinFlywheels(flywheelVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
