// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {

  public enum LEDMode {

    RAINBOW,
    SOLID_ORANGE,
    FLASHING_ORANGE

  }

  AddressableLED leds;
  AddressableLEDBuffer buffer;

  SendableChooser<LEDMode> ledModeChooser;

  ShuffleboardTab teleopTab;
  
  /** Creates a new LEDs. */
  public LEDs() {
    leds = new AddressableLED(Constants.kLEDs.LED_PWM_PORT);
    buffer = new AddressableLEDBuffer(Constants.kLEDs.LED_LENGTH);

    ledModeChooser = new SendableChooser<LEDMode>();

    teleopTab = Shuffleboard.getTab("Teleop Tab");

    ledModeChooser.setDefaultOption("Rainbow", LEDMode.RAINBOW);

    ledModeChooser.addOption("Solid Orange", LEDMode.SOLID_ORANGE);
    ledModeChooser.addOption("Flashing Orange", LEDMode.FLASHING_ORANGE);

    teleopTab.add("LED Mode", ledModeChooser).
    withPosition(3, 1).
    withSize(3, 1);

    leds.setLength(Constants.kLEDs.LED_LENGTH);
    leds.start();
  }

  /**
   * Sets all LEDs along the robot to the same color
   * @param color the color the leds will display
   */
  public void setAll(Color color) {
    Color newColor = new Color(color.red, color.blue, color.green);
    for (int i = 0; i < Constants.kLEDs.LED_LENGTH; i++) {
      buffer.setLED(i, newColor);
    }
    leds.setData(buffer);
  }

  /**
   * Sets all LEDs along the robot to the same color
   * @param r red part of the color the leds will display
   * @param g green part of the color the leds will display
   * @param b blue part of the color the leds will display
   */
  public void setAll(int r, int g, int b) {
    for (int i = 0; i < Constants.kLEDs.LED_LENGTH; i++) {
      buffer.setRGB(i, r, g, b);
    }
    leds.setData(buffer);
  }

  public void setAllHSV(int h, int s, int v) {
    for (int i = 0; i < Constants.kLEDs.LED_LENGTH; i++) {
      buffer.setHSV(i, h, s, v);
    }
    leds.setData(buffer);
  }

  public void setLEDhsv(int index, int h, int s, int v) {
    buffer.setHSV(index, h, s, v);
  }

  public void ledBuffer() {
    leds.setData(buffer);
  }

  public LEDMode getSelectedMode() {

    return ledModeChooser.getSelected();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}