package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.Constants;
import frc.slicelibs.util.math.OnboardModuleState;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {

  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  public final int moduleNumber;

  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController driveFeedback;
  private final PIDController angleFeedback;
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double velocitySetpoint = null; // Setpoint for closed loop control, null for open loop
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  private final boolean isReal;

  public SwerveModule(SwerveModuleIO io, int moduleNumber) {
    this.io = io;
    this.moduleNumber = moduleNumber;

    isReal = RobotBase.isReal();

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.CURRENT_MODE) {
      case REAL:
        driveFeedforward = new SimpleMotorFeedforward(
          Constants.kDrivetrain.DRIVE_KS, Constants.kDrivetrain.DRIVE_KV, Constants.kDrivetrain.DRIVE_KA);
        driveFeedback = new PIDController(0.0, 0.0, 0.0);
        angleFeedback = new PIDController(0.0, 0.0, 0.0);
        break;
      case REPLAY:
        driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        driveFeedback = new PIDController(0.05, 0.0, 0.0);
        angleFeedback = new PIDController(7.0, 0.0, 0.0);
        angleFeedback.enableContinuousInput(-Math.PI, Math.PI);
        break;
      case SIM:
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
        driveFeedback = new PIDController(0.1, 0.0, 0.0);
        angleFeedback = new PIDController(10.0, 0.0, 0.0);
        angleFeedback.enableContinuousInput(-Math.PI, Math.PI);
        break;
      default:
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
        driveFeedback = new PIDController(0.0, 0.0, 0.0);
        angleFeedback = new PIDController(0.0, 0.0, 0.0);
        break;
    }

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
      if (isReal) {
        io.setDriveVelocity(velocitySetpoint, driveFeedforward.calculate(velocitySetpoint));
      }
      else {
        io.setDriveVoltage(driveFeedback.calculate(inputs.driveVelocityMetersPerSec, velocitySetpoint) +
          driveFeedforward.calculate(velocitySetpoint));
      }
    }

    if (angleSetpoint != null) {
      if (isReal) {
        io.setAnglePosition(angleSetpoint.getDegrees());
      }
      else {
        io.setAngleVoltage(angleFeedback.calculate(inputs.anglePosition.getDegrees(), angleSetpoint.getDegrees()));
      }
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
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
    angleSetpoint = optimizedState.angle;
    if (isOpenLoop) {
      io.setDriveVoltage(optimizedState.speedMetersPerSecond / Constants.kDrivetrain.MAX_LINEAR_VELOCITY);
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
    return new SwerveModuleState(velocitySetpoint, angleSetpoint);
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the drive velocity in meters/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityMetersPerSec;
  }

}