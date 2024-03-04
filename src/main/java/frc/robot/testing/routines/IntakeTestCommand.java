// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeTestCommand extends Command {
  /** Creates a new IntakeTestCommand. */
  private Intake m_intake;
  private double[] m_currents;
  private double m_averageCurrent;
  private double m_maxCurrent;
  private double[] m_speeds;
  private double m_averageSpeed;
  public IntakeTestCommand(Intake intake) {
    addRequirements(intake);
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.intakeEntrance.set(1);
    m_intake.intakeRamp.set(1);

    m_currents[0] = m_intake.intakeEntrance.getOutputCurrent();
    m_currents[1] = m_intake.intakeRamp.getOutputCurrent();
    m_maxCurrent = m_currents[0];
    for (int i = 0; i < m_currents.length; i++){
      m_averageCurrent += m_currents[i];
      if (m_currents[i] > m_maxCurrent){
        m_maxCurrent = m_currents[i];
      }
    }
    m_speeds[0] = m_intake.intakeEntranceEncoder.getVelocity();
    m_speeds[1] = m_intake.intakeRampEncoder.getVelocity();
    for(int i = 0; i < m_speeds.length; i++){
      m_averageCurrent += m_speeds[i];
    }
    m_intake.intakeEntrance.set(0);
    m_intake.intakeRamp.set(0);

    m_averageCurrent /= m_currents.length;
    m_averageSpeed /= m_speeds.length;

    System.out.println("Intake Tests:");
    System.out.println("Average Current: " + m_averageCurrent);
    System.out.println("Max Current: " + m_maxCurrent);
    System.out.println("Average Speed: " + m_averageSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
