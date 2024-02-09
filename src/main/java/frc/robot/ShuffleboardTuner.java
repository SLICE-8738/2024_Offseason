// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Creates a Shuffleboard layout for tuning one or more double variables. */
public class ShuffleboardTuner {

    /**
     * Creates a new tuner on Shuffleboard for a single double variable.
     * 
     * @param updateCallback A double consumer to update the code with the value
     *                       set on Shuffleboard for the tuner variable. 
     * @param layoutName The name of the Shuffleboard tuner layout to be created.
     */
    public static void create(DoubleConsumer updateCallback, String layoutName) {

        ShuffleboardLayout tunerLayout = Shuffleboard.getTab("Debug Tab").getLayout(layoutName, BuiltInLayouts.kGrid);

        GenericEntry valueEntry = tunerLayout.add("Value", 0).getEntry();
        tunerLayout.add("Update", new InstantCommand(() -> updateCallback.accept(valueEntry.getDouble(0))));
        
    }

    /**
     * Creates a new tuner on Shuffleboard for mutiple double variables that share one update callback.
     * 
     * @param updateCallback A double array consumer to update the code with the values
     *                       set on Shuffleboard for the tuner variables.
     * @param entryNames The names of the entries representing the double variables to be tuned.
     * @param layoutName The name of the Shuffleboard tuner layout to be created.
     */
    public static void create(Consumer<Double[]> updateCallback, String[] entryNames, String layoutName) {

        ShuffleboardLayout tunerLayout = Shuffleboard.getTab("Debug Tab").getLayout(layoutName, BuiltInLayouts.kGrid);
        GenericEntry[] valueEntries = new GenericEntry[entryNames.length];

        for(int i = 0; i < entryNames.length; i++) {

            valueEntries[i] = tunerLayout.add(entryNames[i], 0).withPosition(0, i).getEntry();

        }

        tunerLayout.add("Update", new InstantCommand(() -> updateCallback.accept(getEntriesAsDoubles(valueEntries))));

    }

    private static Double[] getEntriesAsDoubles(GenericEntry[] entries) {

        Double[] doubleValues = new Double[entries.length];

        for(int i = 0; i < entries.length; i++) {

            doubleValues[i] = entries[i].getDouble(0);

        }

        return doubleValues;

    }

}
