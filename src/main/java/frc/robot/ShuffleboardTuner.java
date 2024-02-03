// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Creates a Shuffelboard layout for tuning one or more double variables. */
public class ShuffleboardTuner {

    /**
     * Creates a new ShuffleboardTuner for a single double variable.
     * 
     * @param updateCallback A double consumer to update the code with the value
     *                       set on Shuffleboard for the tuner variable. 
     * @param layoutName The name of the Shuffleboard tuner layout to be created.
     */
    public ShuffleboardTuner(DoubleConsumer updateCallback, String layoutName) {

        ShuffleboardLayout debugTab = Shuffleboard.getTab("Debug Tab").getLayout(layoutName);

        SimpleWidget valueWidget = debugTab.add("Value", 0);
        debugTab.add("Update", new InstantCommand(() -> updateCallback.accept(valueWidget.getEntry().getDouble(0))));
        
    }

    /**
     * Creates a new ShuffleboardTuner for mutiple double variables that share one callback.
     * 
     * @param updateCallback A double array consumer to update the code with the values
     *                       set on Shuffleboard for the tuner variables.
     * @param entryNames The names of the entries representing the double variables.
     * @param layoutName The name of the Shuffleboard tuner layout to be created.
     */
    public ShuffleboardTuner(Consumer<Double[]> updateCallback, String[] entryNames, String layoutName) {

        ShuffleboardLayout tunerLayout = Shuffleboard.getTab("Debug Tab").getLayout(layoutName, BuiltInLayouts.kGrid);
        Double[] widgetValues = new Double[entryNames.length];

        for(int i = 0; i < entryNames.length; i++) {

            widgetValues[i] = tunerLayout.add(entryNames[i], 0).withPosition(0, i).getEntry().getDouble(0);

        }

        tunerLayout.add("Update", new InstantCommand(() -> updateCallback.accept(widgetValues)));

    }

}
