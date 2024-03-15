// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.LimelightBase;

public class ShooterLimelight extends LimelightBase {

  /** Creates a new ShooterLimelight. */
  public ShooterLimelight() {

    super("limelight-shooter");

  }

  @Override
  public void periodic() {

    targetDetected = table.getEntry("tv").getDouble(0);

    targetXOffset = table.getEntry("tx").getDouble(0);
    targetYOffset = table.getEntry("ty").getDouble(0);

    currentBotPoseBlue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);

    if(currentBotPoseBlue.length != 0) {

      lastBotPoseBlue = currentBotPoseBlue;

    }
    
    currentRobotTargetSpacePose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    handleRawPose(currentRobotTargetSpacePose, lastRobotTargetSpacePose);

    currentTargetCameraSpacePose = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[] {0, 0, 1.178, 0, 0, 0});

    handleRawPose(currentTargetCameraSpacePose, lastTargetCameraSpacePose);

    currentAprilTagID = table.getEntry("tid").getDouble(0);

    if(currentAprilTagID != 0) {

      lastAprilTagID = currentAprilTagID;

    }

  }

}
