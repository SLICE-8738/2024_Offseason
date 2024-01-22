// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.slicelibs.PositionalSubsystem;

public class Elevator extends PositionalSubsystem {
  /** Creates a new Elevator. */
  public Elevator() {
    super(new int[]{0, 0}, new boolean[]{true, false}, 0.1, 0, 0, 1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
