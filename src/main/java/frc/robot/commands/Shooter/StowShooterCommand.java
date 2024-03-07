// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

/**
 * Begins making the shooter angle so that it can pick up a note from the intake
 */
public class StowShooterCommand extends Command {

  private final Shooter m_shooter;

  /** Creates a new SpinUp. */
  public StowShooterCommand(Shooter shooter) {
    m_shooter = shooter;
    
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the shooter angle to stowed
    m_shooter.aimShooter(Constants.kShooter.SHOOTER_STOW_ANGLE + 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
