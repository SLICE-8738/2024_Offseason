// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

public class RunIndexerCommand extends Command {

  private final Indexer m_indexer;
  private final double percentOutput;

  /** Creates a new RunIndexerCommand. */
  public RunIndexerCommand(Indexer indexer, double percentOutput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    m_indexer = indexer;
    this.percentOutput = percentOutput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_indexer.spinIndex(percentOutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_indexer.spinIndex(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_indexer.getOutputCurrent() >= Constants.kIndexer.CURRENT_THRESHOLD;
    
  }
}
