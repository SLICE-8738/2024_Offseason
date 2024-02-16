// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing.routines;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasicMotorTest extends Command {
  /** Creates a new BasicMotorTest. */
  SubsystemBase m_subsystembase;
  DoubleConsumer m_driveFunction;
  DoubleSupplier m_voltage;
  DoubleSupplier m_speed;

  public BasicMotorTest(SubsystemBase subsystemBase, DoubleConsumer driveFunction, DoubleSupplier voltage, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystemBase);
    m_subsystembase = subsystemBase;
    m_driveFunction = driveFunction;
    m_voltage = voltage;
    m_speed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveFunction.accept(1);
    m_voltage.getAsDouble();
    m_speed.getAsDouble();
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
