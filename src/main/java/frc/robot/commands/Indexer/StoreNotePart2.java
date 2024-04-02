// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
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
public class StoreNotePart2 extends Command {
  // creates private variables
  private final Indexer indexer;
  private final Intake intake;

  private final Timer storeTimer, totalTimer;

  private boolean timerRunning;

  private final PIDController pid;

  private boolean outputCurrentThreshold;

  private boolean forceStop;

  public StoreNotePart2(Indexer indexer, Intake intake) {
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
    SmartDashboard.putBoolean("Intaking", true);
    storeTimer.reset();
    storeTimer.stop();

    totalTimer.restart();

    outputCurrentThreshold = false;
    forceStop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.runIntakeEntranceOnly(Constants.kIntake.INTAKE_SPEED);
    intake.runRampIntakeOnly(1/3.0);

    if (totalTimer.get() < 0.15) {
      indexer.spinIndex(-1);
      return;
    }
    // spins the motors
    double distance = indexer.getLaserCanDistance();
    if (indexer.laserCanOnline()) {
      if (distance > Constants.kIndexer.DEFAULT_LASERCAN_DISTANCE) {
        indexer.spinIndex(0.2);
      } else if (Math.abs(distance - Constants.kIndexer.STORE_NOTE_TARGET) < Constants.kIndexer.STORE_NOTE_ERROR_TOLERANCE) {
        indexer.spinIndex(0);
      } else {
        double output = pid.calculate(distance, Constants.kIndexer.STORE_NOTE_TARGET);
        indexer.spinIndex(-output);
      }
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

    if (!indexer.laserCanOnline()) {
      return true;
    }

    if (forceStop) {
      return true;
    }
    
    if (Button.circle1.getAsBoolean() || Button.circle2.getAsBoolean() || Button.rightTrigger1.getAsBoolean() || Button.cross1.getAsBoolean() || Button.rightTrigger2.getAsBoolean()) {
      return true;
    }
    // ends the command
    if (indexer.isStored() && timerRunning && storeTimer.get() > 0.1) {
      return true; // ends the command if stored is true (stored is a method in indexer)
    } else {
      return false;
    }
  }
}
