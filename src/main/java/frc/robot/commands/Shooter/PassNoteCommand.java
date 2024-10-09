// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  boolean aim;
  boolean timerStarted;

  public static final boolean PASS_NOTE_TEST_MODE = false;
  private static final double TEST_TIME = 1.8;
  private Timer testTimer;

  public PassNoteCommand(Shooter shooter, Indexer indexer) {
    this.shooter = shooter;
    this.indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(indexer);

    aim = false;
    timerStarted = false;

    timer = new Timer();

    if (PASS_NOTE_TEST_MODE) {
      testTimer = new Timer();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aim = false;
    timerStarted = false;
    timer.reset();
    timer.stop();

    if (PASS_NOTE_TEST_MODE) {
      testTimer.restart();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean buttonReleased = !Button.leftBumper2.getAsBoolean();
    shooter.spinFlywheels(PASS_NOTE_TEST_MODE ? 1000 : 4500, false);
    boolean ready = false;

    if (PASS_NOTE_TEST_MODE) {
      ready = testTimer.get() > TEST_TIME;
    }else {
      ready = shooter.atTargetSpeed(Constants.kShooter.FLYWHEEL_RPM_ACCEPTABLE_ERROR);
    }


    if (buttonReleased && !aim && ready) {
      aim = true;
    }



    if (aim) {
      shooter.aimShooter(15);
      if (shooter.detectShooterAngle(Constants.kShooter.VERTICAL_AIM_ACCEPTABLE_ERROR) || !indexer.isStored()) {
        if (!timerStarted) {
          timer.start();
          timerStarted = true;
        }
      }

      if (timerStarted) {
        indexer.spinIndex(1);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.spinIndex(0);
    shooter.aimShooter(Constants.kShooter.SHOOTER_STOW_ANGLE + 0.5);
    shooter.spinFlywheels(0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean buttonReleased = !Button.leftBumper2.getAsBoolean();
    if (!aim && buttonReleased && !shooter.atTargetSpeed(Constants.kShooter.FLYWHEEL_RPM_ACCEPTABLE_ERROR)) {
      return true;
    }
    if (timerStarted && timer.get() > 1) {
      return true;
    }
    return false;
  }
}
