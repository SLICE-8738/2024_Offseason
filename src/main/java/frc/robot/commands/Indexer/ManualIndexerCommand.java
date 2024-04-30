// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Indexer;
/**
 Spins the indexer so the note will go into the flywheel, nudges the note into the flywheel
 */
public class ManualIndexerCommand extends Command {

  // Creates private variables
  private final Indexer indexer;
  private final GenericHID controller;

  public ManualIndexerCommand(Indexer indexer, GenericHID controller) {
    //from the indexer subsystem, gets the motor without making a new one 
    addRequirements(indexer);
    this.indexer = indexer;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //starts the spin the motor 
    indexer.spinIndex(-controller.getRawAxis(5) * 0.35);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops the motor 
    indexer.spinIndex(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return indexer.getOutputCurrent() >= Constants.kIndexer.CURRENT_THRESHOLD;
    return false;
  }
}
