// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Add your docs here. */
public class ShuffleboardTuner {

    private final SimpleWidget widget;

    /**
     * Creates a new ShuffelboardTuner.
     * 
     * @param updateCallback A double consumer to update a double variable in the code to the
     *                       value of the Shuffleboard entry.
     * @param entryName The name that the entry should be displayed with on Shuffleboard.
     */
    public ShuffleboardTuner(DoubleConsumer updateCallback, String entryName) {

        ShuffleboardTab debugTab = Shuffleboard.getTab("Debug Tab");
        widget = debugTab.add(entryName, 0);
        debugTab.add("Update " + entryName, new InstantCommand(() -> updateCallback.accept(widget.getEntry().getDouble(0))));
        
    }
}
