package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class SignalStoreNote extends Command {
    private final LEDs leds;
    private final Intake intake;
    private final Indexer indexer;
    private double m_rainbowFirstPixelHue = 0;
    private int range = 9;
    private int color;
    private boolean strobing;
    private int strobingCounter;

    
    public SignalStoreNote(LEDs leds, Indexer indexer, Intake intake) {
      this.leds = leds;
      this.indexer = indexer;
      this.intake = intake;

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(leds);
    }

    @Override
    public void initialize() {
      leds.setAll(Color.kBlack);
    }

    @Override
    public void execute() {
      strobing = indexer.isStored();
      color = strobing ? 60 : 2;
        for(int i = 0; i < Constants.kLEDs.LED_LENGTH; i++) {
            final int hue = ((((int)m_rainbowFirstPixelHue + (4 * i * range / Constants.kLEDs.LED_LENGTH )) % range) + color) % 180;
            if (strobing) {
              leds.setLEDhsv(i, hue, 255, ((strobingCounter + i) % 32) >= 16 ? 128 : 0);
            } //else if(intake.getRampOutputCurrent() > 0){
              //leds.setLEDhsv(i, 0, 100, 100);
            //} else if(LimelightHelpers.getTV("Limelight-Intake") == true && indexer.isStored() == false){
              //leds.setLEDhsv(i, 45, 100 , (strobingCounter +i) % 32 >= 16 ? 128 : 0);
            //}
            else {
              leds.setLEDhsv(i, hue, 255, 128);
            }
        }

        m_rainbowFirstPixelHue += 0.25;
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