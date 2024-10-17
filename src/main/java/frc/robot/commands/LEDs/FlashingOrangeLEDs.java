// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class FlashingOrangeLEDs extends Command {
  
  private final LEDs m_LEDs;
  private final Intake m_Intake;
  private final Timer timer;

  /** Creates a new FlashingOrangeLEDs. */
  public FlashingOrangeLEDs(LEDs leds, Intake intake) {
    m_LEDs = leds; 
    m_Intake = intake;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LEDs);
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() % 2 == 0) {
      m_LEDs.setAllHSV(175, 255, 128);
    }else if (timer.get() % 2 != 0) {
      m_LEDs.setAll(Color.kBlack);
    }
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
