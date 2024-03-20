// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Begins making the shooter flywheels slow down to a complete stop
 */
public class SpinDownCommand extends Command {

  private final Shooter m_shooter;

  /** Creates a new SpinUp. */
  public SpinDownCommand(Shooter shooter) {
    m_shooter = shooter;
    // No requirements needed since it uses PIDs
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the flywheel speed to 0
    m_shooter.spinFlywheels(0, false);
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
