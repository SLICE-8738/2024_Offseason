package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class CustomRainbowLEDs extends Command {
    private final LEDs leds;
    private int m_rainbowFirstPixelHue = 0;
    private int range = 20;
    private int color;
    
    public CustomRainbowLEDs(LEDs leds, int hue) {
      this.leds = leds;
      color = hue;

      // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(leds);
    }

    @Override
    public void initialize() {
      leds.setAll(Color.kBlack);
    }

    @Override
    public void execute() {
        for(int i = 0; i < 77; i++) {
            final int hue = (((m_rainbowFirstPixelHue + (i * range / 77 )) % range) + color) % 180;
            leds.setLEDhsv(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 0.5;
        m_rainbowFirstPixelHue %= range;

        leds.ledBuffer();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      leds.setAll(Color.kBlack);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}