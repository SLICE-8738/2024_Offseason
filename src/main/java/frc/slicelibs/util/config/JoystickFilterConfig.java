package frc.slicelibs.util.config;

import frc.slicelibs.PolarJoystickFilter;

/**
 * Contains the configuration values for a Joystick filter
 * @see PolarJoystickFilter
 */
public class JoystickFilterConfig {
    public final double deadzone, exponent, exponentPercent, smoothing;

    /**
     * Create a new PolarJoystickConfig
     * @param deadzone any joystick input udner this ammount will be set to 0
     * @param smoothing the maximum amount the joystick input can change in seconds
     * @param exponent the exponent of the polynomial curve the joystick input will be put through 
     * @param exponentPercent what percentage of the polynomial will use the exponent parameter instead of a linear function
     */
    public JoystickFilterConfig(double deadzone, double smoothing, double exponent, double exponentPercent) {
        this.deadzone = deadzone;
        this.smoothing = smoothing;
        this.exponent = exponent;
        this.exponentPercent = exponentPercent;
    }

    /**
     * Create a new PolarJoystickConfig
     * @param deadzone any joystick input udner this ammount will be set to 0
     * @param smoothing the maximum amount the joystick input can change in seconds
    */
    public JoystickFilterConfig(double deadzone, double smoothing) {
        this.deadzone = deadzone;
        this.smoothing = smoothing;
        this.exponent = 1;
        this.exponentPercent = 1;
    }

    /**
     * Create a new PolarJoystickConfig
     * @param deadzone any joystick input udner this ammount will be set to 0
    */
    public JoystickFilterConfig(double deadzone) {
        this.deadzone = deadzone;
        this.smoothing = 0;
        this.exponent = 1;
        this.exponentPercent = 1;
    }
}
