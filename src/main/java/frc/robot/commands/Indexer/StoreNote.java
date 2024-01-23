// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/** 
Store the note in the intake and runs both the intake and ramp intake
*/
public class StoreNote extends Command {
  //creates private variables 
  private final Indexer indexer;
  private final Intake intake;


  public StoreNote(Indexer indexer, Intake intake) {
    //from the indexer and intake subsystems, gets the motors without making a new one 
    addRequirements(indexer);
    addRequirements(intake);
    this.indexer = indexer;
    this.intake = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //spins the motors

    indexer.spinIndex(.5);
    intake.runRampIntakeOnly(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops the motors
    indexer.spinIndex(0);
    intake.runRampIntakeOnly(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //ends the command
    if (indexer.isStored()) {
      return true; //ends the command if stored is true (stored is a constructor in indexer)
    } else {
      return false;
    }
  }
}
