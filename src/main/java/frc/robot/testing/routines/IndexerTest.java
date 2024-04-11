// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing.routines;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IndexerTest extends Command {
  /** Creates a new indexerTestCommand. */
  private Indexer indexer;
  private double averageCurrent = 0;
  private double maxCurrent = 0;
  private double averageSpeed = 0;
  private int executes = 0;
  private final Timer timer;

  private ShuffleboardTab testRoutine;
  private SimpleWidget indexerCurrentWidget, indexerVelocityWidget;
  
  public IndexerTest(Indexer indexer) {
  
  addRequirements(indexer);
  this.indexer = indexer;
  timer = new Timer();

  testRoutine = Shuffleboard.getTab("Test Routine");
  indexerCurrentWidget = testRoutine.add("Top Flywheel Current", 0);
  indexerVelocityWidget = testRoutine.add("Top Flywheel Velocity", 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.spinIndex(0.5);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    executes += 1;

    averageCurrent += indexer.getOutputCurrent();

    if (FullTestRoutine.USE_SHUFFLEBOARD) {
      indexerCurrentWidget.getEntry().setDouble(indexer.getOutputCurrent());
    }


    if(indexer.getOutputCurrent() > maxCurrent){
      maxCurrent = indexer.getOutputCurrent();
    }

    averageSpeed += indexer.getVelocity();

    // Outputting to Shuffleboard
    indexerVelocityWidget.getEntry().setDouble(indexer.getVelocity());


  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    indexer.spinIndex(0);

    for(int i = 0; i < 2; i++){
      averageCurrent /= executes;
    }

    System.out.println("indexer Tests");
    System.out.println(String.format("Average Current: %s", averageCurrent));
    System.out.println(String.format("Max Current: %s", maxCurrent));
    System.out.println(String.format("Average Speed: %s", averageSpeed));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 4;
  }
}
