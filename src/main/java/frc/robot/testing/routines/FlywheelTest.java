// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing.routines;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class FlywheelTest extends Command {
  /** Creates a new ShooterTestCommand. */
  private Shooter shooter;
  private double[] averageCurrent = {0,0};
  private double[] maxCurrent = {0,0};
  private double[] averageSpeed = {0,0};
  private int executes = 0;
  private final Timer timer;

  private ShuffleboardTab testRoutine;
  private SimpleWidget flywheelTopCurrentWidget, flywheelBottomCurrentWidget, flywheelTopVelocityWidget, flywheelBottomVelocityWidget;
  
  public FlywheelTest(Shooter shooter) {
  
  addRequirements(shooter);
  this.shooter = shooter;
  timer = new Timer();

  testRoutine = Shuffleboard.getTab("Test Routine");
  flywheelTopCurrentWidget = testRoutine.add("Top Flywheel Current", 0);
  flywheelBottomCurrentWidget = testRoutine.add("Bottom Flywheel Current", 0);
  flywheelTopVelocityWidget = testRoutine.add("Top Flywheel Velocity", 0);
  flywheelBottomVelocityWidget = testRoutine.add("Bottom flywheel; Velocity", 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.flywheelTop.set(0.25);
    shooter.flywheelBottom.set(0.25);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    executes += 1;

    averageCurrent[0] += shooter.getTopOutputCurrent();
    averageCurrent[1] += shooter.getBottomOutputCurrent();

    if (FullTestRoutine.USE_SHUFFLEBOARD) {
      flywheelTopCurrentWidget.getEntry().setDouble(shooter.getTopOutputCurrent());
      flywheelBottomCurrentWidget.getEntry().setDouble(shooter.getBottomOutputCurrent());
    }


    if(shooter.getTopOutputCurrent() > maxCurrent[0]){
      maxCurrent[0] = shooter.getTopOutputCurrent();
    }
    if(shooter.getBottomOutputCurrent() > maxCurrent[1]){
      maxCurrent[1] = shooter.getBottomOutputCurrent();
    }

    averageSpeed[0] += shooter.getTopSpeed();
    averageSpeed[1] += shooter.getBottomSpeed();

    // Outputting to Shuffleboard
    flywheelTopVelocityWidget.getEntry().setDouble(shooter.getTopSpeed());
    flywheelBottomVelocityWidget.getEntry().setDouble(shooter.getBottomSpeed());


  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    shooter.flywheelTop.set(0);
    shooter.flywheelBottom.set(0);

    for(int i = 0; i < 2; i++){
      averageCurrent[i] /= executes;
      averageSpeed[i] /= executes;
    }

    System.out.println("Shooter Tests");
    System.out.println(String.format("Average Current: \nTop Motor: %s \nBottom Motor: %s", averageCurrent[0], averageCurrent[1]));
    System.out.println(String.format("Max Current: \nTop Motor: %s \nBottom Motor: %s", maxCurrent[0], maxCurrent[1]));
    System.out.println(String.format("Average Speed: \nTop Motor: %s \nBottom Motor: %s", averageSpeed[0], averageSpeed[1]));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 4;
  }
}
