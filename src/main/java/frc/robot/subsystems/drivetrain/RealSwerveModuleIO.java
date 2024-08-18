package frc.robot.subsystems.drivetrain;

import java.util.OptionalDouble;
import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.slicelibs.util.config.REVConfigs;
import frc.slicelibs.util.config.SwerveModuleConstants;
import frc.slicelibs.util.factories.SparkMaxFactory;
import frc.slicelibs.util.math.Conversions;

public class RealSwerveModuleIO implements SwerveModuleIO {
  private final Rotation2d angleOffset;

  private final TalonFX driveMotor;
  private final CANSparkMax angleMotor;
  private final CANcoder angleEncoder;
  private final RelativeEncoder integratedAngleEncoder;
  private final SparkPIDController angleFeedback;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> angleAbsolutePosition;
  private final Queue<Double> anglePositionQueue;

  public RealSwerveModuleIO(SwerveModuleConstants moduleConstants) {
    this.angleOffset = moduleConstants.angleOffset;

    driveMotor = new TalonFX(moduleConstants.driveMotorID);
    angleMotor = SparkMaxFactory.createSparkMax(moduleConstants.angleMotorID, REVConfigs.angleSparkMaxConfig);
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleFeedback = angleMotor.getPIDController();

    driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
    driveMotor.getConfigurator().setPosition(0);

    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

    integratedAngleEncoder.setPositionConversionFactor(Constants.kDrivetrain.ANGLE_POSITION_CONVERSION_FACTOR_DEGREES);
    integratedAngleEncoder.setVelocityConversionFactor(Constants.kDrivetrain.ANGLE_VELOCITY_CONVERSION_FACTOR_DEGREES);
    resetToAbsolute();
    
    timestampQueue = OdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveMotor.getPosition();
    drivePositionQueue =
        OdometryThread.getInstance().registerSignal(() -> OptionalDouble.of(driveMotor.getPosition().getValueAsDouble()));
    driveVelocity = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getSupplyCurrent();

    angleAbsolutePosition = angleEncoder.getAbsolutePosition();
    anglePositionQueue =
        OdometryThread.getInstance().registerSignal(() -> OptionalDouble.of(integratedAngleEncoder.getPosition()));

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kDrivetrain.ODOMETRY_FREQUENCY_HZ, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kDrivetrain.DRIVE_DEFAULT_FRAME_RATE_HZ,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent);
    //riveMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent);

    inputs.drivePositionMeters =
        Conversions.talonToMeters(drivePosition.getValueAsDouble(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO);
    inputs.driveVelocityMetersPerSec =
        Conversions.talonToMPS(driveVelocity.getValueAsDouble(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO);
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.angleAbsolutePosition =
        Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble());
    inputs.anglePosition =
        Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    inputs.angleVelocityDegreesPerSec =
        integratedAngleEncoder.getVelocity();
    inputs.angleAppliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
    inputs.angleCurrentAmps = new double[] {angleMotor.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Conversions.talonToMeters(value, Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO))
            .toArray();
    inputs.odometryAnglePositions =
        anglePositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    anglePositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveVelocity(double velocity, double feedforward) {
    driveMotor.setControl(new VelocityVoltage(velocity).withFeedForward(feedforward));
  }

  @Override
  public void setAngleVoltage(double volts) {
    angleMotor.setVoltage(volts);
  }

  @Override
  public void setAnglePosition(double position) {
    angleFeedback.setReference(position, ControlType.kPosition);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveMotor.setNeutralMode(enable? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setAngleBrakeMode(boolean enable) {
    angleMotor.setIdleMode(enable? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void resetToAbsolute() {
    integratedAngleEncoder.setPosition(angleEncoder.getAbsolutePosition().waitForUpdate(250).getValue() * 360 - angleOffset.getDegrees());
  }
}