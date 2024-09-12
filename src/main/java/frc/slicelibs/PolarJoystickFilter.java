
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.slicelibs;

import frc.slicelibs.util.config.JoystickFilterConfig;

/** Add your docs here. */
public class PolarJoystickFilter {

    private double lastInput;
    private double lastThetaValue;
    private final double deadzone, smoothing, exponent, exponentPercent;
    private final boolean curve;

    public PolarJoystickFilter(JoystickFilterConfig config) {
        this.deadzone = config.deadzone;
        this.smoothing = config.smoothing;
        this.exponent = config.exponent;
        this.exponentPercent = config.exponentPercent;
        this.curve = true;
        
        lastInput = 0;
    }

    public double[] toPolar(double rawX, double rawY) {

        if (rawX == 0 && rawY == 0) {
            return new double[] {0, 0};
        }
        
        double[] polarCoords = {
            Math.atan2(rawY, rawX),
            Math.sqrt(rawX * rawX + rawY * rawY)};

        lastThetaValue = polarCoords[0];

        if (polarCoords[1] > 1) {
            polarCoords[1] = 1;
        }

        return polarCoords;
    }

    public double[] withDead(double[] polarCoords) {
        if(polarCoords[1] < deadzone) {
            return new double[] {lastThetaValue, 0};
        }
        else {
            return polarCoords;
        }
    }

    public double withCurve(double raw) {
        double firstTerm = exponentPercent * Math.pow(Math.abs(raw), exponent);
        firstTerm = Math.copySign(firstTerm, raw);
        double secondTerm = (1 - exponentPercent) * raw;
        return firstTerm + secondTerm;
    }

    public double[] filter(double rawX, double rawY) {
        double[] filtered = toPolar(rawX, rawY);
        filtered = withDead(filtered);
        if (curve) {
            filtered[1] = withCurve(filtered[1]);
        }
        filtered[1] = smoothing * lastInput + (1 - smoothing) * filtered[1];
        lastInput = filtered[1];
        double[] signal = withDead(filtered);
        return new double[] {Math.cos(signal[0]) * signal[1], Math.sin(signal[0]) * signal[1]};
    }

}