// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Button;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Store the note in the intake and runs both the intake and ramp intake
 */
public class StoreNote extends Command {
  // creates private variables
  private final Indexer indexer;
  private final Intake intake;

  private final Timer timer;

  private boolean timerRunning;

  public StoreNote(Indexer indexer, Intake intake) {
    // from the indexer and intake subsystems, gets the motors without making a new
    // one
    addRequirements(indexer);
    addRequirements(intake);
    this.indexer = indexer;
    this.intake = intake;

    timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // spins the motors
    double distance = indexer.getLaserCanDistance();
    if (distance < 50) {
      indexer.spinIndex(-.1);
    } else if (distance > 150 && distance < 300) {
      indexer.spinIndex(.1);
    } else if (distance > 300) {
      indexer.spinIndex(.3);
    } else {
      indexer.spinIndex(0);
    }

    intake.runIntake(.5);

    boolean stored = indexer.isStored();
    if (stored && !timerRunning) {
      timer.restart();
    }else if (!stored && timerRunning) {
      timer.reset();
      timer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stops the motors
    indexer.spinIndex(0);
    intake.runIntake(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Button.circle1.getAsBoolean() || Button.circle2.getAsBoolean() || Button.rightTrigger1.getAsBoolean() || Button.cross1.getAsBoolean() || Button.rightTrigger2.getAsBoolean()) {
      return true;
    }
    // ends the command
    if (indexer.isStored() && timerRunning && timer.get() > 0.5) {
      return true; // ends the command if stored is true (stored is a method in indexer)
    } else {
      return false;
    }
  }
}
