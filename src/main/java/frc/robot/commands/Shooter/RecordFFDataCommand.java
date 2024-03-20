// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RecordFFDataCommand extends Command {

  private final Shooter m_shooter;
  private double voltage;
  private final Timer timer;
  private List<Double> speeds;
  private double speed;
  /** Creates a new RecordFFDataCommand. */
  public RecordFFDataCommand(Shooter shooter) {
    m_shooter = shooter;
    timer = new Timer();

    speeds = new ArrayList<Double>();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    voltage = 0;

    timer.restart();
    m_shooter.voltageSpinFlywheels(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > 0.25) {
      timer.restart();
      voltage += 0.1;
      speed = m_shooter.getFlywheelSpeed();
      m_shooter.voltageSpinFlywheels(voltage);
      speeds.add(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.voltageSpinFlywheels(0);
    String message = "";
    for (double s : speeds) {
      message += s + "\n";
    }
    System.out.println(message);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return voltage >= 12 || speed >= 8000;
  }
}
