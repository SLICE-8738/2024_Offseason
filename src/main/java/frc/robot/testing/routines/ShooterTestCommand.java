// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing.routines;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterTestCommand extends Command {
  /** Creates a new ShooterTestCommand. */
  private Shooter m_shooter;
  private double[] m_currents;
  private double m_averageCurrent;
  private double m_maxCurrent;
  private double[] m_speeds;
  private double m_averageSpeed;
  public ShooterTestCommand(Shooter shooter) {
  addRequirements(shooter);
  m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.flywheelTop.set(1);
    m_shooter.flywheelBottom.set(1);
    m_shooter.aimMotor.set(1);
    m_currents[0] = m_shooter.flywheelTop.getOutputCurrent();
    m_currents[1] = m_shooter.flywheelBottom.getOutputCurrent();
    m_currents[2] = m_shooter.aimMotor.getOutputCurrent();
    m_maxCurrent = m_currents[0];
    for (int i = 0; i < m_currents.length; i++){
      m_averageCurrent += m_currents[i];
      if (m_currents[i] > m_maxCurrent){
        m_maxCurrent = m_currents[i];
      }
    }
    m_speeds[0] = m_shooter.flyTopEncoder.getVelocity();
    m_speeds[1] = m_shooter.flyBottomEncoder.getVelocity();
    m_speeds[2] = m_shooter.aimEncoder.getVelocity();
    for(int i = 0; i < m_speeds.length; i++){
      m_averageCurrent += m_speeds[i];
    }
    m_shooter.flywheelTop.set(0);
    m_shooter.flywheelBottom.set(0);
    m_shooter.aimMotor.set(0);
    
    m_averageCurrent /= m_currents.length;
    m_averageSpeed /= m_speeds.length;

    System.out.println("Shooter Tests");
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
