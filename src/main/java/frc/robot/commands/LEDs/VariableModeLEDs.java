package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDMode;

public class VariableModeLEDs extends Command {
    private final LEDs m_leds;
    private int m_rainbowFirstPixelHue = 0;

    private final Timer timer;
    private boolean on;

    private LEDMode ledMode;
    private LEDMode lastLEDMode;

    public VariableModeLEDs(LEDs leds) {

      m_leds = leds;

      timer = new Timer();

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(leds);
    }

    @Override
    public void initialize() {
      m_leds.setAll(Color.kBlack);

      timer.reset();
      timer.start();
      on = false;
    }

    @Override
    public void execute() {

      ledMode = m_leds.getSelectedMode();

      if(ledMode != lastLEDMode) {

        lastLEDMode = m_leds.getSelectedMode();

        m_leds.setAll(Color.kBlack);

      }

      if(ledMode == LEDMode.RAINBOW) {

        for(int i = 0; i < 77; i++) {
          final int hue = (m_rainbowFirstPixelHue + (i * 180 / 77 )) % 180;
          m_leds.setLEDhsv(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 1.5;
        m_rainbowFirstPixelHue %= 180;

        m_leds.ledBuffer();

      }
      else if(ledMode == LEDMode.SOLID_ORANGE) {

        m_leds.setAllHSV(175, 255, 128);

      }
      else {

        if ((timer.get() % 1) > 0.5 && !on) {
          m_leds.setAllHSV(175, 255, 128);
          on = true;
        }else if ((timer.get() % 1) <= 0.5 && on) {
          m_leds.setAll(Color.kBlack);
          on = false;
        }

      }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_leds.setAll(Color.kBlack);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
}