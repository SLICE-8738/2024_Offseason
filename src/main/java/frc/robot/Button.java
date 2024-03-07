package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Button {

    /* Controllers */
    public static PS4Controller controller1 = new PS4Controller(0);
    public static GenericHID controller2 = new GenericHID(1);

    /* Controller 1 Buttons */
    public static Trigger square1 = new JoystickButton(controller1, 1); //Controller 1 Square
    public static Trigger cross1 = new JoystickButton(controller1, 2); //Controller 1 Cross
    public static Trigger circle1 = new JoystickButton(controller1, 3); //Controller 1 Circle
    public static Trigger triangle1 = new JoystickButton(controller1, 4); //Controller 1 Triangle
    public static Trigger leftBumper1 = new JoystickButton(controller1, 5); //Controller 1 Left Bumper
    public static Trigger rightBumper1 = new JoystickButton(controller1, 6); //Controller 1 Right Bumper
    public static Trigger leftTrigger1 = new JoystickButton(controller1, 7); //Controller 1 Left Trigger
    public static Trigger rightTrigger1 = new JoystickButton(controller1, 8); //Controller 1 Right Trigger
    public static Trigger share = new JoystickButton(controller1, 9); //Controller 1 Share
    public static Trigger options = new JoystickButton(controller1, 10); //Controller 1 Options
    public static Trigger leftStickClick1 = new JoystickButton(controller1, 11); //Controller 1 Left Stick Click
    public static Trigger rightStickClick1 = new JoystickButton(controller1, 12); //Controller 1 Right Stick Click
    public static Trigger psButton1 = new JoystickButton(controller1, 13); //Controller 1 PS Button
    public static Trigger touchPad1 = new JoystickButton(controller1, 14); //Controller 1 Touch Pad
    public static Trigger controlPadRight1 = new POVButton(controller1, 90); //Controller 1 Control Pad Right
    public static Trigger controlPadUp1 = new POVButton(controller1, 0); //Controller 1 Control Pad Up
    public static Trigger controlPadLeft1 = new POVButton(controller1, 270); //Controller 1 Control Pad Left
    public static Trigger controlPadDown1 = new POVButton(controller1, 180); //Controller 1 Control Pad Down

    /* Controller 2 Buttons */
    public static Trigger square2 = new JoystickButton(controller2, 1); //Controller 2 X
    public static Trigger cross2 = new JoystickButton(controller2, 2); //Controller 2 A
    public static Trigger circle2 = new JoystickButton(controller2, 3); //Controller 2 B
    public static Trigger triangle2 = new JoystickButton(controller2, 4); //Controller 2 Y
    public static Trigger leftBumper2 = new JoystickButton(controller2, 5); //Controller 2 Left Bumper
    public static Trigger rightBumper2 = new JoystickButton(controller2, 6); //Controller 2 Right Bumper
    public static Trigger leftTrigger2 = new JoystickButton(controller2, 7); //Controller 2 Left Trigger
    public static Trigger rightTrigger2 = new JoystickButton(controller2, 8); //Controller 2 Right Trigger
    public static Trigger back = new JoystickButton(controller2, 9); //Controller 2 Back
    public static Trigger start = new JoystickButton(controller2, 10); //Controller 2 Start
    public static Trigger leftStickClick2 = new JoystickButton(controller2, 11); //Controller 2 Left Stick Click
    public static Trigger rightStickClick2 = new JoystickButton(controller2, 12); //Controller 2 Right Stick Click
    public static Trigger controlPadRight2 = new POVButton(controller2, 90); //Controller 2 Control Pad Right
    public static Trigger controlPadUp2 = new POVButton(controller2, 0); //Controller 2 Control Pad Up
    public static Trigger controlPadLeft2 = new POVButton(controller2, 270); //Controller 2 Control Pad Left
    public static Trigger controlPadDown2 = new POVButton(controller2, 180); //Controller 2 Control Pad Down

}
