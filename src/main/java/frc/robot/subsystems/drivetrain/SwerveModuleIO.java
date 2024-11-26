// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public Rotation2d absoluteAnglePosition = new Rotation2d();
    public Rotation2d integratedAnglePosition = new Rotation2d();
    public double angleVelocityDegreesPerSec = 0.0;
    public double angleAppliedVolts = 0.0;
    public double angleCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /** Run the drive motor at the specified duty cycle percent output. */
  public default void setDriveDutyCycle(double percentOutput) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Set the given drive motor velocity in meters/second as a PID setpoint.*/
  public default void setDriveVelocity(double velocity) {}

  /** Run the angle motor at the specified duty cycle percent output. */
  public default void setAngleDutyCycle(double percentOutput) {}

  /** Run the angle motor at the specified voltage. */
  public default void setAngleVoltage(double volts) {}

  /** Set the given angle motor position in degrees as a PID setpoint. */
  public default void setAnglePosition(double position) {}

  /** 
   * Reset the integrated angle encoder position to
   * the position of the CANcoder with the offset.
   */
  public default void resetToAbsolute() {}
}