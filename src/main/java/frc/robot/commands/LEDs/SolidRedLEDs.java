package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class SolidRedLEDs extends Command {
  private final LEDs leds;
  private final Intake intake;

  /** Creates a new SolidOrangeLEDS. */
  public SolidRedLEDs(LEDs leds, Intake intake) {
    this.leds = leds;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getRampOutputCurrent() > 0) {
        leds.setAllHSV(0, 255, 255);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
