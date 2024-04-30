// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class ReverseWhileNoteStoredCommand extends Command {

  Intake m_intake;
  Indexer m_indexer;
  GenericHID operatorController;
  /** Creates a new ReverseWhileNoteStoredCommand. */
  public ReverseWhileNoteStoredCommand(Intake intake, Indexer indexer, GenericHID operatorController) {
    m_intake = intake;
    m_indexer = indexer;
    this.operatorController = operatorController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_indexer.isStored()) {
      if (Math.abs(operatorController.getRawAxis(5)) < 0.1) {
        m_intake.runIntake(0);
      }else {
        m_intake.runIntakeEntranceOnly(-operatorController.getRawAxis(5) * 0.3);
        m_intake.runRampIntakeOnly(-operatorController.getRawAxis(5) * 0.35);
      }
      
    }else {
        m_intake.runIntakeEntranceOnly(-operatorController.getRawAxis(5) * 0.3);
        m_intake.runRampIntakeOnly(-operatorController.getRawAxis(5) * 0.58);
    }
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
