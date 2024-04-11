// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Button;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class PassNoteCommand extends Command {
  /** Creates a new PassNoteCommand. */

  private final Shooter shooter;
  private final Indexer indexer;
  Timer timer;
  boolean timerStarted;

  private static final boolean TEST_MODE = false;
  private static final double TEST_TIME = 3;
  private Timer testTimer;

  public PassNoteCommand(Shooter shooter, Indexer indexer) {
    this.shooter = shooter;
    this.indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(indexer);
    timerStarted = false;

    timer = new Timer();

    if (TEST_MODE) {
      testTimer = new Timer();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerStarted = false;
    timer.reset();
    timer.stop();

    if (TEST_MODE) {
      testTimer.restart();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean buttonReleased = !Button.leftBumper2.getAsBoolean();
    shooter.spinFlywheels(TEST_MODE ? 1000 : 4250, false);

    if (TEST_MODE) {
      if (!timerStarted && testTimer.get() > TEST_TIME) {
        timerStarted = true;
      }
    }

    if (buttonReleased && !timerStarted && shooter.atTargetSpeed(Constants.kShooter.FLYWHEEL_RPM_ACCEPTABLE_ERROR)) {
      timerStarted = true;
      timer.start();
    }



    if (timerStarted) {
      indexer.spinIndex(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.spinIndex(0);
    shooter.spinFlywheels(0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean buttonReleased = !Button.leftBumper2.getAsBoolean();
    if (!timerStarted && buttonReleased && !shooter.atTargetSpeed(Constants.kShooter.FLYWHEEL_RPM_ACCEPTABLE_ERROR)) {
      return true;
    }
    if (timerStarted && timer.get() > 1) {
      return true;
    }
    return false;
  }
}
