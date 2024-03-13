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
public class StoreNote extends Command {
  // creates private variables
  private final Indexer indexer;
  private final Intake intake;

  private final Timer timer;

  private boolean timerRunning;

  private final PIDController pid;

  public StoreNote(Indexer indexer, Intake intake) {
    // from the indexer and intake subsystems, gets the motors without making a new
    // one
    addRequirements(indexer);
    addRequirements(intake);
    this.indexer = indexer;
    this.intake = intake;

    timer = new Timer();

    pid = new PIDController(Constants.kIndexer.STORE_NOTE_KP, 0, 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Intaking", true);
    timer.reset();
    timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // spins the motors
    double distance = indexer.getLaserCanDistance();

    if (indexer.isStored()) {
      indexer.spinIndex(0);
    } else if (distance > Constants.kIndexer.DEFAULT_LASERCAN_DISTANCE) {
      indexer.spinIndex(0.3);
    } else {
      double output = pid.calculate(distance, Constants.kIndexer.STORE_NOTE_TARGET);
      indexer.spinIndex(-output);
    }

    intake.runIntakeEntranceOnly(Constants.kIntake.INTAKE_SPEED);
    intake.runRampIntakeOnly(1/3.0);

    boolean stored = indexer.isStored();
    if (stored && !timerRunning) {
      timer.restart();
      timerRunning = true;
    }else if (!stored && timerRunning) {
      timer.reset();
      timer.stop();
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
