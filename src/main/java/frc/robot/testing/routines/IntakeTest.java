// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing.routines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeTest extends Command {
  /** Creates a new IntakeTestCommand. */
  private Intake intake;
  private double[] averageCurrent = {0,0};
  private double[] maxCurrent = {0,0};
  private double[] averageSpeed = {0,0};
  private int executes = 0;
  private final Timer timer;
  
  public IntakeTest(Intake intake) {
    addRequirements(intake);
    this.intake = intake;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intakeEntrance.set(1);
    intake.intakeRamp.set(1);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    executes += 1;
    averageCurrent[0] += intake.intakeEntrance.getOutputCurrent();
    averageCurrent[1] += intake.intakeRamp.getOutputCurrent();
    if(intake.intakeEntrance.getOutputCurrent() > maxCurrent[0]){
      maxCurrent[0] = intake.intakeEntrance.getOutputCurrent();
    }
    if(intake.intakeRamp.getOutputCurrent() > maxCurrent[1]){
      maxCurrent[1] = intake.intakeRamp.getOutputCurrent();
    }
    averageSpeed[0] = intake.intakeEntranceEncoder.getVelocity();
    averageSpeed[1] = intake.intakeRampEncoder.getVelocity();
    
    if(timer.get() >= 2){
      isFinished();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeEntrance.set(0);
    intake.intakeRamp.set(0);

    for(int i = 0; i < 2; i++){
      averageCurrent[i] /= executes;
      averageSpeed[i] /= executes;
    }

    System.out.println("Intake Tests:");
    System.out.println(String.format("Average Current: \nIntake Entrance: %s \nIntake Ramp: %s", averageCurrent[0], averageCurrent[1]));
    System.out.println(String.format("Max Current: \nIntake Enterance: %s \nIntake Ramp %s", maxCurrent[0], maxCurrent[1]));
    System.out.println(String.format("Average Speed: \nIntake Enterance: %s \nIntake Ramp: %s", averageSpeed[0], averageSpeed[1]));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
