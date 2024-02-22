// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Class for interfacing with a Limelight connected to NetworkTables. */
public class LimelightTable {

  private final NetworkTable table;
  
  private double targetDetected;

  private double targetXOffset;
  private double targetYOffset;

  private double[] currentBotPoseBlue = new double[0];
  private double[] lastBotPoseBlue = new double[6];
  private double[] currentRobotTargetSpacePose = new double[6];
  private double[] lastRobotTargetSpacePose = new double[6];
  private double[] currentTargetCameraSpacePose = new double[6];
  private double[] lastTargetCameraSpacePose = new double[6];

  private double currentAprilTagID;
  private double lastAprilTagID;

  private final NetworkTableEntry ledMode;
  private final NetworkTableEntry cameraMode;
  private final NetworkTableEntry pipeline;

  /**
   * Creates a new Limelight.
   * 
   * @param tableKey The key/name assigned to the desired Limglight on NetworkTables.
   */
  public LimelightTable(String tableKey) {
    
    table = NetworkTableInstance.getDefault().getTable(tableKey);

    ledMode = table.getEntry("ledMode");
    cameraMode = table.getEntry("cameraMode");
    pipeline = table.getEntry("pipeline");

  }

  /** Updates vision data from AprilTags. */
  public void update() {

    targetDetected = table.getEntry("tv").getDouble(0);

    targetXOffset = table.getEntry("tx").getDouble(0);
    targetYOffset = table.getEntry("ty").getDouble(0);

    currentBotPoseBlue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);

    handleRawPose(currentBotPoseBlue, lastBotPoseBlue);

    currentRobotTargetSpacePose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    handleRawPose(currentRobotTargetSpacePose, lastRobotTargetSpacePose);

    currentTargetCameraSpacePose = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

    handleRawPose(currentTargetCameraSpacePose, lastTargetCameraSpacePose);

    currentAprilTagID = table.getEntry("tid").getDouble(0);

    if(currentAprilTagID != 0) {

      lastAprilTagID = currentAprilTagID;

    }

  }

  /**
   * @return Whether the Limelight can see any valid targets
   */
  public boolean getTargetDetected() {

    return targetDetected == 1;

  }

  /**
   * @return The horizontal offset from the crosshair to the target
   */
  public double getXOffset() {

    return targetXOffset;

  }

  /**
   * @return The vertical offset from the crosshair to the target
   */
  public double getYOffset() {

    return targetYOffset;

  }

  /**
   * @return The last received non-empty robot pose with the origin at the right-hand side of the blue alliance driverstation
   *         if any. All-zero pose if none has been received yet.
   */
  public Pose2d getLastBotPoseBlue() {

    return new Pose2d(lastBotPoseBlue[0], lastBotPoseBlue[1], Rotation2d.fromDegrees(lastBotPoseBlue[5]));

  }

  /**
   * @return The current robot pose with the origin at the right-hand side of the blue alliance driverstation
   *         being received if any. Null if received pose is empty.
   */
  public Pose2d getCurrentBotPoseBlue() {
      
    double[] currentBotPoseBlue = this.currentBotPoseBlue;
 
    if(currentBotPoseBlue.length != 0) {

      return new Pose2d(currentBotPoseBlue[0], currentBotPoseBlue[1], Rotation2d.fromDegrees(currentBotPoseBlue[5]));

    }
    else {

      return null;

    }

  }

  /**
   * @return The last received non-empty robot pose with the target as the origin if any.
   *         All-zero pose if none has been received yet.
   */
  public Pose2d getRobotTargetSpacePose() {

    return new Pose2d(lastRobotTargetSpacePose[0], lastRobotTargetSpacePose[1], Rotation2d.fromDegrees(lastRobotTargetSpacePose[5]));

  }

  /**
   * @return The last received non-empty camera pose with the target as the origin if any.
   *         All-zero pose if none has been received yet.
   */
  public Pose2d getTargetCameraSpacePose() {

    return new Pose2d(lastTargetCameraSpacePose[0], lastTargetCameraSpacePose[1], Rotation2d.fromDegrees(lastTargetCameraSpacePose[5]));

  }

  /**
   * @return The ID of the last identified primary in-view AprilTag
   */
  public double getAprilTagID() {

    return lastAprilTagID;

  }

  /**
   * Sets the Limelight's LED state.
   * 
   * @param mode
   * 
   * <p>0: Use the LED mode set in the current pipeline.
   * <p>1: Force off
   * <p>2: Force blink
   * <p>3: Force on
   */
  public void setLedMode(Number mode) {

    ledMode.setNumber(mode);

  }
  
  /**
   * Sets the Limelight's operation mode.
   * 
   * @param mode
   * 
   * <p>0: Vision processor
   * <p>1: Driver camera (Increases exposure, disables vision processing)
   */
  public void setCameraMode(Number mode) {

    cameraMode.setNumber(mode);

  }

  /**
   * Sets the Limelight's current pipeline.
   * 
   * @param pipelineNumber The desired pipeline number (0-9)
   */
  public void setPipeline(Number pipelineNumber) {

    pipeline.setNumber(pipelineNumber);

  }

  public void handleRawPose(double[] rawPose, double[] processedPose) {

    if(rawPose != new double[6]) {

      processedPose = rawPose;

    }

  }

}