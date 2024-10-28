package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;
import frc.slicelibs.util.math.Conversions;

public class SimSwerveModuleIO implements SwerveModuleIO {
  private final DCMotorSim driveSim = new DCMotorSim(DCMotor.getKrakenX60(1), Constants.kDrivetrain.DRIVE_GEAR_RATIO, 0.032); // This moment of inertia is a rough estimate for now;
  private final DCMotorSim angleSim = new DCMotorSim(DCMotor.getNEO(1), Constants.kDrivetrain.ANGLE_GEAR_RATIO, 0.004); // This moment of inertia is a rough estimate for now;

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 2.0);
  private final PIDController driveFeedback = new PIDController(0.1, 0.0, 0.0);
  private final PIDController angleFeedback = new PIDController(0.5, 0.0, 0.0);
  
  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double angleAppliedVolts = 0.0;

  public SimSwerveModuleIO() {
    angleFeedback.enableContinuousInput(-180, 180);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    driveSim.update(0.02);
    angleSim.update(0.02);

    inputs.drivePositionMeters = driveSim.getAngularPositionRotations() * Constants.kDrivetrain.WHEEL_CIRCUMFERENCE;
    inputs.driveVelocityMetersPerSec = Conversions.RPMToMPS(driveSim.getAngularVelocityRPM(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};

    inputs.angleAbsolutePosition =
        new Rotation2d(angleSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.anglePosition = new Rotation2d(angleSim.getAngularPositionRad());
    inputs.angleVelocityDegreesPerSec = angleSim.getAngularVelocityRadPerSec() * 180.0 / Math.PI;
    inputs.angleAppliedVolts = angleAppliedVolts;
    inputs.angleCurrentAmps = new double[] {Math.abs(angleSim.getCurrentDrawAmps())};

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsMeters = new double[] {inputs.drivePositionMeters};
    inputs.odometryAnglePositions = new Rotation2d[] {inputs.anglePosition};
  }

  @Override
  public void setDriveVelocity(double velocity) {
    driveAppliedVolts = MathUtil.clamp(
      driveFeedback.calculate(
        Conversions.RPMToMPS(driveSim.getAngularVelocityRPM(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE), 
        velocity) + driveFeedforward.calculate(velocity), 
      -12, 
      12);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setAnglePosition(double position) {
    angleAppliedVolts = MathUtil.clamp(
      angleFeedback.calculate(
        angleSim.getAngularPositionRotations() * 360, 
        position
      ), 
      -12.0, 
      12.0);
    angleSim.setInputVoltage(angleAppliedVolts);
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setAngleVoltage(double volts) {
    angleAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    angleSim.setInputVoltage(angleAppliedVolts);
  }
}