// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.slicelibs;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.slicelibs.util.config.JoystickFilterConfig;

/** 
 * The polar joystick filter applies a deadzone, polynomial curve, and slew rate limiter to joystick inputs
 * It does this to the magnitude of the joystick vector, rather than the individual x and y components
 * This maintains the joystick's which is useful for swerve drive.
 */
public class PolarJoystickFilter {

    JoystickFilterConfig config;
    double lastInput;
    SlewRateLimiter slewRateLimiter;

    ShuffleboardTab filterTab;

    public PolarJoystickFilter(JoystickFilterConfig config) {

        this.config = config;
        
        lastInput = 0;

        slewRateLimiter = new SlewRateLimiter(config.maxAcceleration);

    }

    /**
     * Converts a set of rectangular coordinates into polar coordinates
     * @param rawX the x coordinate of the joystick input
     * @param rawY the y coordinate of the joystick input
     * @return a 2 long array, with the first value being the angle of the polar coordinate and the second value being the radius/magnitude
     */
    private double[] toPolar(double rawX, double rawY) {

        if (rawX == 0 && rawY == 0) {
            return new double[] {0, 0};
        }
        
        double[] polarCoords = {
            Math.atan2(rawY, rawX),
            Math.sqrt(rawX * rawX + rawY * rawY)};

        if (polarCoords[1] > 1) {
            polarCoords[1] = 1;
        }

        return polarCoords;
    }

    /**
     * Converts a set of poalr coordinates into rectangular coordinates
     * @param theta the angle of the coordinate
     * @param r the magnitude of the coordinate
     * @return a 2 long array, with the first value being the x coordinate and the second value being the y coordinate
     */
    private double[] toRectangular(double theta, double r) {
        return new double[] {Math.cos(theta) * r, Math.sin(theta) * r};
    }


    /**
     * Applies a deadzone to the joystick input, to prevent joystick drive
     * @param polarCoords the polar coordinate sof the joystick input
     * @return new polar coordinates with a deadzone applied to the magnitude
     */
    private double[] withDead(double[] polarCoords) {
        if(polarCoords[1] < config.deadzone) {
            return new double[] {polarCoords[0], 0};
        }
        else {
            return polarCoords;
        }
    }

    /**
     * Applies a polynomial curve to the joystick input to make movement more/less sensitive at lower speeds
     * @param raw the magnitude of the joystick input
     * @return the new magnitude value
     */
    private double withCurve(double raw) {
        double firstTerm = config.exponentPercent * Math.pow(Math.abs(raw), config.exponent);
        firstTerm = Math.copySign(firstTerm, raw);
        double secondTerm = (1 - config.exponentPercent) * raw;
        return firstTerm + secondTerm;
    }

    /**
     * Limits hows quickly the joystick input can change to prevent instantaneous starts/stops
     * @param raw the magnitude of the joystick input
     * @return the new magnitude value
     */
    private double withSlewRateLimiter(double raw) {
        return slewRateLimiter.calculate(raw);
    }

    /**
     * Applies a deadzone, polynomial input curve, and slew rate limiter to the joystick input
     * @param rawX x value of raw joystick input
     * @param rawY y value of raw joystick input
     * @return a 2 long array containing the x and y value of the filtered joystick value
     */
    public double[] filter(double rawX, double rawY) {
        // Convert joystick coordinates to polar coordinates
        double[] filtered = toPolar(rawX, rawY);

        // Apply filters
        filtered = withDead(filtered);
        filtered[1] = withCurve(filtered[1]);
        filtered[1] = withSlewRateLimiter(filtered[1]);
        double[] signal = withDead(filtered);
        
        return toRectangular(signal[0], signal[1]);
    }

}