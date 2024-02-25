package frc.slicelibs.util.config;

import frc.slicelibs.PolarJoystickFilter;

/**
 * Contains the configuration values for a Joystick filter
 * @see PolarJoystickFilter
 */
public class JoystickFilterConfig {
    public final double deadzone, exponent, exponentPercent, maxAcceleration;

    /**
     * Create a new PolarJoystickConfig
     * @param deadzone any joystick input udner this ammount will be set to 0
     * @param maxAcceleration the maximum amount the joystick input can change in seconds
     * @param exponent the exponent of the polynomial curve the joystick input will be put through 
     * @param exponentPercent what percentage of the polynomial will use the exponent parameter instead of a linear function
     */
    public JoystickFilterConfig(double deadzone, double maxAcceleration, double exponent, double exponentPercent) {
        this.deadzone = deadzone;
        this.maxAcceleration = maxAcceleration;
        this.exponent = exponent;
        this.exponentPercent = exponentPercent;
    }

    /**
     * Create a new PolarJoystickConfig
     * @param deadzone any joystick input udner this ammount will be set to 0
     * @param maxAcceleration the maximum amount the joystick input can change in seconds
    */
    public JoystickFilterConfig(double deadzone, double maxAcceleration) {
        this.deadzone = deadzone;
        this.maxAcceleration = maxAcceleration;
        this.exponent = 1;
        this.exponentPercent = 1;
    }

    /**
     * Create a new PolarJoystickConfig
     * @param deadzone any joystick input udner this ammount will be set to 0
    */
    public JoystickFilterConfig(double deadzone) {
        this.deadzone = deadzone;
        this.maxAcceleration = 1000;
        this.exponent = 1;
        this.exponentPercent = 1;
    }
}
