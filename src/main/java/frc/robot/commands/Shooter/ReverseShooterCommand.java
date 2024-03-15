// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/**
 * Reverses the shooter flywheels and indexer until a note has completely passed through the indexer
 * This can be used to intake a note from the source instead of the ground
 */
public class ReverseShooterCommand extends Command {

  private final Shooter m_shooter;
  private final Indexer m_indexer;
  private boolean noteDetected; // Whether a note has passed in front of the indexer's LaserCAN

  /** Creates a new ReverseShooterCommand. */
  public ReverseShooterCommand(Shooter shooter, Indexer indexer) {
    m_shooter = shooter;
    m_indexer = indexer;
    noteDetected = false;
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
    // If a note is detected as stored, remember this
    if (!noteDetected && m_indexer.isStored()) {
      noteDetected = true;
    } 

    // Run the flywheels and indexer in reverse
    m_shooter.dutyCycleSpinFlywheels(-0.5);
    m_indexer.spinIndex(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stops the command when the note was detected before but isn't any longer
    // This means it has completely passed through the shooter flywheels into the indexer
    return noteDetected && !m_indexer.isStored();
  }
}
