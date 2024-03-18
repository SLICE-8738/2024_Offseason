package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDs;

public class SignalStoreNote extends Command {
    private final LEDs leds;
    private final Indexer indexer;
    private double m_rainbowFirstPixelHue = 0;
    private int range = 7;
    private int color;
    private boolean strobing;
    private int strobingCounter;
    
    public SignalStoreNote(LEDs leds, Indexer indexer) {
      this.leds = leds;
      this.indexer = indexer;

      // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(leds);
    }

    @Override
    public void initialize() {
      leds.setAll(Color.kBlack);
    }

    @Override
    public void execute() {
      strobing = indexer.getLaserCanDistance() < Constants.kIndexer.DEFAULT_LASERCAN_DISTANCE;
      color = strobing ? 130 : 170;
        for(int i = 0; i < Constants.kLEDs.LED_LENGTH; i++) {
            final int hue = ((((int)m_rainbowFirstPixelHue + (i * range / Constants.kLEDs.LED_LENGTH )) % range) + color) % 180;
            if (strobing) {
              leds.setLEDhsv(i, hue, 255, (strobingCounter + i) % 32);
            }else {
              leds.setLEDhsv(i, hue, 255, 128);
            }
        }

        m_rainbowFirstPixelHue += 0.1;
        m_rainbowFirstPixelHue %= range;
        strobingCounter += 2;
        strobingCounter %= 32;

        leds.ledBuffer();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      for(int i = 0; i < Constants.kLEDs.LED_LENGTH; i++) {
        leds.setLEDhsv(i, 0, 0, 0);
    }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}