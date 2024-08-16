// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.slicelibs.PolarJoystickFilter;
import frc.slicelibs.util.config.JoystickFilterConfig;

public class ManualShooterCommand extends Command {

  private final Shooter m_shooter;
  private final Indexer m_indexer;
  private final Drivetrain m_drivetrain;
  private final GenericHID m_operatorController;
  private final PolarJoystickFilter speedFilter;

  private final double BASE_FLYWHEEL_SPEED = 0;

  /** Creates a new AimShooterCommand. */
  public ManualShooterCommand(Shooter shooter, Drivetrain drivetrain, Indexer indexer, GenericHID operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    m_shooter = shooter;
    m_indexer = indexer;
    m_drivetrain = drivetrain;
    m_operatorController = operatorController;

    speedFilter = new PolarJoystickFilter(new JoystickFilterConfig(0.08));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double aimSpeed = speedFilter.filter(-m_operatorController.getRawAxis(1), 0)[0];
    double flywheelSpeed = m_drivetrain.atAllianceWing() && m_indexer.isStored() ? BASE_FLYWHEEL_SPEED : 0;

    if (Math.abs(aimSpeed) > 0.1 || !m_shooter.pidAimControl || m_shooter.getAlternateAngle() > 120 || m_shooter.detectShooterAngle(Constants.kShooter.VERTICAL_AIM_ACCEPTABLE_ERROR)) {
      m_shooter.dutyCycleAimShooter(aimSpeed);
    }

    if (-m_operatorController.getRawAxis(5) > -0.1) {
      m_shooter.spinFlywheels(flywheelSpeed, false);
    } else {
      m_shooter.dutyCycleSpinFlywheels(-m_operatorController.getRawAxis(5) * 0.1);
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
    return false;
  }
}
