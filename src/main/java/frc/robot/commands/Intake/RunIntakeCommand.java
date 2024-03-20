// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Button;
import frc.robot.subsystems.Intake;

/**
 * Runs the intake wheels (including the vectoring ramp wheels) in order to pick up a note from the ground
 */
public class RunIntakeCommand extends Command {
  private final Intake m_intake;
  private final double percentOutput;
  /** Creates a new SpinIntakeCommand. */
  public RunIntakeCommand(Intake intake, double percentOutput) {
    m_intake = intake;
    this.percentOutput = percentOutput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runIntake(percentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Button.circle2.getAsBoolean();
  }
}
