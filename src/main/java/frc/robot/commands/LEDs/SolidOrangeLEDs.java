// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDMode;

public class SolidOrangeLEDs extends Command {
  private final LEDs leds;
  private final Intake intake;

  private int m_soildOrangeFirstPixelHue;
  /** Creates a new SolidOrangeLEDS. */
  public SolidOrangeLEDs(LEDs leds, Intake intake) {
    this.leds = leds;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getEntranceOutputCurrent() > 0) {
        leds.setAllHSV(175, 255, 128);
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
