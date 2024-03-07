// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Button;
import frc.robot.subsystems.Shooter;

public class ClimbLockCommand extends Command {
  Shooter m_shooter;
  Timer lockTimer;
  GenericHID overrideController;

  // Time the lock button must be held in order to lock the 
  private final double LOCK_TIME = 0.5;

  /** Creates a new ClimbLockCommand. */
  public ClimbLockCommand(Shooter shooter, GenericHID overrideController) {
    m_shooter = shooter;
    this.overrideController = overrideController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);

    lockTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lockTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lockTimer.get() > LOCK_TIME) {
      m_shooter.dutyCycleAimShooter(-1);
    } else {
      m_shooter.dutyCycleAimShooter(-overrideController.getRawAxis(1));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.dutyCycleAimShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (lockTimer.get() < LOCK_TIME) {
      return !Button.leftTrigger2.getAsBoolean();
    }
    return (-overrideController.getRawAxis(1)) > 0.5;
  }
}
