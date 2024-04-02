// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing.routines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
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

  private double[] currentAverage = {5.352, 0.02};
  private double[] currentMax = {22.531, 0.375};
  private double[] speedAverage = {4.118, 4.464};

  private ShuffleboardTab testingTab;
  private SimpleWidget intakeEntranceCurrentWidget, intakeRampCurrentWidget, intakeEntranceVelocityWidget, intakeRampVelocityWidget; 
  
  public IntakeTest(Intake intake) {
    addRequirements(intake);
    this.intake = intake;
    timer = new Timer();

    // Shuffleboard elements
    testingTab = Shuffleboard.getTab("Test Routine");
    intakeEntranceCurrentWidget = testingTab.add("Intake Entrance Motor Current", 0);
    intakeRampCurrentWidget = testingTab.add("Intake Ramp Current", 0);
    intakeEntranceVelocityWidget = testingTab.add("Intake Entrance Velocity", 0);
    intakeRampVelocityWidget = testingTab.add("Intake Ramp Velocity", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.runIntakeEntranceOnly(0.25);
    intake.runRampIntakeOnly(0.25);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    executes += 1;
    averageCurrent[0] += intake.getEntranceOutputCurrent();
    averageCurrent[1] += intake.getRampOutputCurrent();

    // Outputting to Shuffleboard
    intakeEntranceCurrentWidget.getEntry().setDouble(intake.getEntranceOutputCurrent());
    intakeRampCurrentWidget.getEntry().setDouble(intake.getRampOutputCurrent());

    if(intake.getEntranceOutputCurrent() > maxCurrent[0]){
      maxCurrent[0] = intake.getEntranceOutputCurrent();
    }
    if(intake.getRampOutputCurrent() > maxCurrent[1]){
      maxCurrent[1] = intake.getRampOutputCurrent();
    }
    averageSpeed[0] = intake.getEntranceVelocity();
    averageSpeed[1] = intake.getRampVelocity();

    // Outputting to Shuffleboard
    intakeEntranceVelocityWidget.getEntry().setDouble(intake.getEntranceVelocity());
    intakeRampVelocityWidget.getEntry().setDouble(intake.getRampVelocity());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntakeEntranceOnly(0);
    intake.runRampIntakeOnly(0);;

    for(int i = 0; i < 2; i++){
      averageCurrent[i] /= executes;
      averageSpeed[i] /= executes;
    }

    /*if(averageCurrent[0]){
      
    }*/

    System.out.println("Intake Tests:");
    System.out.println("Deviation from average entrance current is: %s  %s\nRamp deviation: %s  %s");
    System.out.println("Deviation from Max entrance current is: %s \nRamp deviation is %s");
    
    System.out.println(String.format("Average Current: \nIntake Entrance: %s \nIntake Ramp: %s", averageCurrent[0], averageCurrent[1]));
    System.out.println(String.format("Max Current: \nIntake Enterance: %s \nIntake Ramp %s", maxCurrent[0], maxCurrent[1]));
    System.out.println(String.format("Average Speed: \nIntake Enterance: %s \nIntake Ramp: %s", averageSpeed[0], averageSpeed[1]));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return timer.get() >= 4;
    }
  }
