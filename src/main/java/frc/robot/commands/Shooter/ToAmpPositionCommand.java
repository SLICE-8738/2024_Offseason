// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ToAmpPositionCommand extends Command {

  private final Shooter m_shooter;
  private final GenericHID operatorController;

  /** Creates a new ToAmpPosition. */
  public ToAmpPositionCommand(Shooter shooter, GenericHID operatorController) {
    m_shooter = shooter;

    this.operatorController = operatorController;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.resetAimSpeed();
    m_shooter.aimShooter(Constants.kShooter.SHOOTER_AMP_SCORE_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.dutyCycleSpinFlywheels(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopFlywheels();
    m_shooter.slowDownAim();
    m_shooter.aimShooter(Constants.kShooter.SHOOTER_STOW_ANGLE + 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(operatorController.getRawAxis(1)) > 0.1;
  }
}
