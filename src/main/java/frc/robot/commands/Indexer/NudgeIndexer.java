// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Indexer;
/**
 Spins the indexer so the note will go into the flywheel, nudges the note into the flywheel
 */
public class NudgeIndexer extends Command {
  // Creates private variables
  private final Indexer indexer;
  private Timer time;
  public NudgeIndexer(Indexer indexer) {
    //from the indexer subsystem, gets the motor without making a new one 
    addRequirements(indexer);
    this.indexer = indexer;
    //creates new timer
    time = new Timer();
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //restarts the timer when the command starts
    time.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //starts the spin the motor 
    indexer.spinIndex(1);
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
    //ends the command
    if (time.get() >= 0.5) {
      return true; //ends command if time greater than or equal to one second
    } else {
      return false;
    }

  }
}
