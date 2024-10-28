package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;
import frc.slicelibs.util.math.OnboardModuleState;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {

  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  public final int moduleNumber;

  private Double velocitySetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d lastAngle;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public SwerveModule(SwerveModuleIO io, int moduleNumber) {
    this.io = io;
    this.moduleNumber = moduleNumber;
    lastAngle = getAngle();
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Drivetrain/Module" + Integer.toString(moduleNumber), inputs);

    if (velocitySetpoint != null) {
      io.setDriveVelocity(velocitySetpoint);
    }

    if (angleSetpoint != null) {
      io.setAnglePosition(angleSetpoint.getDegrees());
    }

    // Calculate positions for odometry
    odometryPositions = new SwerveModulePosition[inputs.odometryTimestamps.length];
    for (int i = 0; i < inputs.odometryTimestamps.length; i++) {
      odometryPositions[i] = new SwerveModulePosition(
        inputs.odometryDrivePositionsMeters[i],
        inputs.odometryAnglePositions[i]);
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state, boolean isOpenLoop) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = OnboardModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    angleSetpoint = (Math.abs(optimizedState.speedMetersPerSecond) <= (Constants.kDrivetrain.MAX_LINEAR_VELOCITY * 0.01))
      ? lastAngle
      : optimizedState.angle;

    lastAngle = angleSetpoint;

    if (isOpenLoop) {
      io.setDriveDutyCycle(optimizedState.speedMetersPerSecond / Constants.kDrivetrain.MAX_LINEAR_VELOCITY);
      velocitySetpoint = null;
    }
    else {
      velocitySetpoint = optimizedState.speedMetersPerSecond;
    }

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop angle control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.setDriveVoltage(volts);
    velocitySetpoint = null;
  }

  /** Runs the module with the specified duty cycle percent outputs. */
  public void runDutyCycle(double drivePercentOutput, double anglePercentOutput) {
    io.setDriveDutyCycle(drivePercentOutput);
    io.setAngleDutyCycle(anglePercentOutput);
    velocitySetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setAngleVoltage(0.0);
    io.setDriveVoltage(0.0);

    // Disable closed loop control for angle and drive
    angleSetpoint = null;
    velocitySetpoint = null;
  }

  /** Sets whether brake mode is enabled for the drive motor. */
  public void setDriveBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
  }

  /** Sets whether brake mode is enabled for the angle motor. */
  public void setAngleBrakeMode(boolean enabled) {
    io.setAngleBrakeMode(enabled);
  }

  public void resetToAbsolute() {
    io.resetToAbsolute();
  }

  /** Returns the current angle of the module measured
   *  by the relative encoder with the offset. */
  public Rotation2d getAngle() {
    return inputs.anglePosition;
  }

  /** Returns the current angle of the module measured
   *  by the CANcoder without the offset.
   */
  public Rotation2d getCANcoderAngle() {
    return inputs.angleAbsolutePosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityMetersPerSec;
  }

  /** Returns the module position (angle position and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (angle position and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the target module state (angle position setpoint and 
   *  drive velocity setpoint). */
  public SwerveModuleState getTargetState() {
    return new SwerveModuleState(
      velocitySetpoint == null? 0 : velocitySetpoint, 
      angleSetpoint == null? new Rotation2d() : angleSetpoint);
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the output current of the drive motor in amps. */
  public double getDriveOutputCurrent() {
    return inputs.driveCurrentAmps[0];
  }

}