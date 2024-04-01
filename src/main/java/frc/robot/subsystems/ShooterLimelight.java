// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.LimelightBase;

public class ShooterLimelight extends LimelightBase {

  /** Creates a new ShooterLimelight. */
  public ShooterLimelight() {

    super("limelight-shooter");
    lastTargetCameraSpacePose = new double[] {0, 0, 1.178, 0, 0, 0};

  }

}
