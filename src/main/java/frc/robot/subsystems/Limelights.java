// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightTable;

public class Limelights extends SubsystemBase {

  private static LimelightTable shooterLimelight, intakeLimelight;

  /** Creates a new Limelights. */
  public Limelights() {

    shooterLimelight = new LimelightTable("limelight-shooter");
    intakeLimelight = new LimelightTable("limelight-intake");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    shooterLimelight.update();
    intakeLimelight.update();

  }

  public static LimelightTable getShooterLimelight() {

    return shooterLimelight;

  }

  public static LimelightTable getIntakeLimelight() {

    return intakeLimelight;

  }

}
