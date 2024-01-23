// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

/**
 * Begins making the shooter flywheels speed up to shooting speed
 */
public class SpinUpCommand extends Command {

  private final Shooter m_shooter;

  /** Creates a new SpinUp. */
  public SpinUpCommand(Shooter shooter) {
    m_shooter = shooter;
    // No requirements needed since it uses PIDs
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the flywheel speed to the appropriate RPM for shooting
    m_shooter.spinFlywheel(Constants.kShooter.FLYWHEEL_RPM);
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
