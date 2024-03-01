// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;
import frc.slicelibs.PolarJoystickFilter;
import frc.slicelibs.util.config.JoystickFilterConfig;

public class ManualShooterCommand extends Command {

  private final Shooter m_shooter;
  private final GenericHID m_operatorController;
  private final PolarJoystickFilter speedFilter;

  private final SimpleWidget flywheelSpeedWidget;

  /** Creates a new AimShooterCommand. */
  public ManualShooterCommand(Shooter shooter, GenericHID operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    m_shooter = shooter;
    m_operatorController = operatorController;

    speedFilter = new PolarJoystickFilter(new JoystickFilterConfig(0.08));

    flywheelSpeedWidget = Shuffleboard.getTab("Debug Tab").add("Set Flywheel Speed", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double aimSpeed = speedFilter.filter(-m_operatorController.getRawAxis(1), 0)[0];
    double flywheelSpeed = flywheelSpeedWidget.getEntry().getDouble(0);

    m_shooter.dutyCycleAimShooter(aimSpeed);
    m_shooter.dutyCycleSpinFlywheel(flywheelSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.dutyCycleAimShooter(0);
    m_shooter.dutyCycleSpinFlywheel(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
