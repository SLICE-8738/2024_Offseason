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

    /* Drivetrain Buttons */
    public static Trigger pathfindToSource = new POVButton(controller1, 0); //Controller 1 Control Pad Right
    public static Trigger pathfindToAmp = new POVButton(controller1, 180); //Controller 1 Control Pad Left

    /* Unassigned Controller 1 Buttons */
    public static Trigger square = new JoystickButton(controller1, 1);
    public static Trigger cross = new JoystickButton(controller1, 2);
    public static Trigger circle = new JoystickButton(controller1, 3);
    public static Trigger triangle = new JoystickButton(controller1, 4);
    public static Trigger leftBumper1 = new JoystickButton(controller1, 5);
    public static Trigger rightBumper1 = new JoystickButton(controller1, 6);
    public static Trigger leftTrigger1 = new JoystickButton(controller1, 7);
    public static Trigger rightTrigger1 = new JoystickButton(controller1, 8);
    public static Trigger share = new JoystickButton(controller1, 9);
    public static Trigger options = new JoystickButton(controller1, 10);
    public static Trigger leftStickClick1 = new JoystickButton(controller1, 11);
    public static Trigger rightStickClick1 = new JoystickButton(controller1, 12);
    public static Trigger psButton1 = new JoystickButton(controller1, 13);
    public static Trigger touchPad1 = new JoystickButton(controller1, 14);
    public static Trigger controlPadUp1 = new POVButton(controller1, 90);
    public static Trigger controlPadDown1 = new POVButton(controller1, 270);

    /* Unassigned Controller 2 Buttons */
    public static Trigger x = new JoystickButton(controller2, 1);
    public static Trigger a = new JoystickButton(controller2, 2);
    public static Trigger b = new JoystickButton(controller2, 3);
    public static Trigger y = new JoystickButton(controller2, 4);
    public static Trigger leftBumper2 = new JoystickButton(controller2, 5);
    public static Trigger rightBumper2 = new JoystickButton(controller2, 6);
    public static Trigger leftTrigger2 = new JoystickButton(controller2, 7);
    public static Trigger rightTrigger2 = new JoystickButton(controller2, 8);
    public static Trigger back = new JoystickButton(controller2, 9);
    public static Trigger start = new JoystickButton(controller2, 10);
    public static Trigger leftStickClick2 = new JoystickButton(controller2, 11);
    public static Trigger rightStickClick2 = new JoystickButton(controller2, 12);
    public static Trigger controlPadRight2 = new POVButton(controller1, 0);
    public static Trigger controlPadUp2 = new POVButton(controller1, 90);
    public static Trigger controlPadLeft2 = new POVButton(controller1, 180);
    public static Trigger controlPadDown2 = new POVButton(controller1, 270);

}
