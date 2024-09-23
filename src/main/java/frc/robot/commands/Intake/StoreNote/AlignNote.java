// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.StoreNote;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Button;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Store the note in the intake and runs both the intake and ramp intake
 */
public class AlignNote extends Command {
  // creates private variables
  private final Indexer indexer;
  private final Intake intake;

  private final Timer storeTimer, totalTimer;

  private boolean timerRunning;

  private final PIDController pid;

  private boolean forceStop;

  public AlignNote(Indexer indexer, Intake intake) {
    // from the indexer and intake subsystems, gets the motors without making a new
    // one
    addRequirements(indexer);
    addRequirements(intake);
    this.indexer = indexer;
    this.intake = intake;

    storeTimer = new Timer();
    totalTimer = new Timer();

    pid = new PIDController(Constants.kIndexer.STORE_NOTE_KP, 0, Constants.kIndexer.STORE_NOTE_KD);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    storeTimer.reset();
    storeTimer.stop();

    totalTimer.restart();

    forceStop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // spins the motors
    double highDistance = indexer.getHighLaserCanDistance();
    double lowDistance = indexer.getLowLaserCanDistance();
    if (indexer.lowLaserCanOnline()) {
      if (lowDistance > 60) {
        indexer.spinIndex(0.15);
        intake.runRampIntakeOnly(0.15 * 1.3333 * 1.5);
      } else if (highDistance < 50) {
        indexer.spinIndex(-0.15);
        intake.runRampIntakeOnly(-0.15 * 1.3333 * 1.5);
      }else if (Math.abs(lowDistance - Constants.kIndexer.STORE_NOTE_TARGET) < Constants.kIndexer.STORE_NOTE_ERROR_TOLERANCE) {
        indexer.spinIndex(0);
        intake.runRampIntakeOnly(0);
      } else {
        double output = pid.calculate(lowDistance, Constants.kIndexer.STORE_NOTE_TARGET);
        indexer.spinIndex(-output);
        if (output < 0) {
          intake.runRampIntakeOnly(-output * 1.3333 * 1.5);
        } else {
          intake.runRampIntakeOnly(0);
        }
        
      }
    }

    boolean stored = indexer.isStored();
    if (stored && !timerRunning) {
      storeTimer.restart();
      timerRunning = true;
    }else if (!stored && timerRunning) {
      storeTimer.reset();
      storeTimer.stop();
      timerRunning = false;
    }
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

    if (!indexer.highLaserCanOnline()) {
      return true;
    }

    if (forceStop) {
      return true;
    }
    
    if (Button.circle1.getAsBoolean() || Button.circle2.getAsBoolean() || Button.rightTrigger1.getAsBoolean() || Button.cross1.getAsBoolean() || Button.rightTrigger2.getAsBoolean()) {
      return true;
    }

    return false;
  }
}
