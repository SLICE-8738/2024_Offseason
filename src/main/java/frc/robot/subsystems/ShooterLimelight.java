// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightTable;

public class ShooterLimelight extends SubsystemBase {

  private static LimelightTable table = new LimelightTable("limelight-shooter");

  /** Creates a new ShooterLimelight. */
  public ShooterLimelight() {}

  @Override
  public void periodic() {

    table.update();

  }

  public static LimelightTable getTable() {

    return table;

  }

}
