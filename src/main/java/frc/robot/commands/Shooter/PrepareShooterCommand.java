// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShooterMath;
import frc.robot.ShooterMath.ShotDetails;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/**
 * Aims and spins up the flywheels to prepare for a shot into the speaker
 */
public class PrepareShooterCommand extends Command {
  Shooter m_shooter;
  Indexer m_indexer;
  Drivetrain m_drivetrain;
  /** Creates a new ShootCommand. */
  public PrepareShooterCommand(Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
    m_shooter = shooter;
    m_indexer = indexer;
    m_drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Figures our the current position
    Translation2d currentPosition = m_drivetrain.getPose().getTranslation();
    // Determines distance to the speaker
    double distanceToSpeaker = currentPosition.getDistance(Constants.kFieldPositions.SPEAKER_POSITION);
    // Uses distance info the calculate optimal shot
    ShotDetails shotDetails = ShooterMath.getShot(distanceToSpeaker);
    // Sets the flywheel speed and aim angle to the appropriate values 
    m_shooter.spinFlywheel(shotDetails.getFlywheelVelocity());
    m_shooter.aimShooter(shotDetails.getShooterAngle());
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
