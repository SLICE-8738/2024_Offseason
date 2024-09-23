// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.StoreNote;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Button;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Store the note in the intake and runs both the intake and ramp intake
 */
public class IntakeNote extends Command {
  // creates private variables
  private final Indexer indexer;
  private final Intake intake;

  private boolean outputCurrentThreshold;

  private boolean forceStop;

  public IntakeNote(Indexer indexer, Intake intake) {
    // from the indexer and intake subsystems, gets the motors without making a new
    // one
    addRequirements(indexer);
    addRequirements(intake);
    this.indexer = indexer;
    this.intake = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outputCurrentThreshold = false;
    forceStop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // spins the motors
    if (indexer.lowLaserCanOnline()) {
      indexer.spinIndex(0.20);
    } else {
      if (!outputCurrentThreshold && indexer.getOutputCurrent() > Constants.kIndexer.CURRENT_THRESHOLD) {
        outputCurrentThreshold = true;
      }

      if (!outputCurrentThreshold) {
        indexer.spinIndex(0.3);
      } else {
        indexer.spinIndex(0);
        forceStop = true;
      }
    }


    intake.runIntakeEntranceOnly(Constants.kIntake.INTAKE_SPEED);
    intake.runRampIntakeOnly(0.65);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stops the motors
    indexer.spinIndex(0);
    intake.runIntake(0);

    SmartDashboard.putBoolean("Intaking", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (forceStop) {
      return true;
    }
    
    if (Button.circle1.getAsBoolean() || Button.circle2.getAsBoolean() || Button.rightTrigger1.getAsBoolean() || Button.cross1.getAsBoolean() || Button.rightTrigger2.getAsBoolean()) {
      return true;
    }
    // ends the command
    return indexer.isStored() && !Button.cross2.getAsBoolean();
  }
}
