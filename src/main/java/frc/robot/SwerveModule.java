package frc.robot;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.slicelibs.util.config.REVConfigs;
import frc.slicelibs.util.config.SwerveModuleConstants;
import frc.slicelibs.util.factories.SparkMaxFactory;
import frc.slicelibs.util.math.Conversions;
import frc.slicelibs.util.math.OnboardModuleState;

public class SwerveModule {

    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d angleAbsolutePosition = new Rotation2d();
        public Rotation2d anglePosition = new Rotation2d();
        public double angleVelocityDegreesPerSec = 0.0;
        public double angleAppliedVolts = 0.0;
        public double angleCurrentAmps = 0.0;
    }

    public final int moduleNumber;
    private final Rotation2d angleOffset;
    private Rotation2d lastAngle;
    private SwerveModuleState targetState = new SwerveModuleState();

    private final TalonFX driveMotor;
    private final CANSparkMax angleMotor;
    private final DCMotorSim driveMotorSim;
    private final DCMotorSim angleMotorSim;
    private final RelativeEncoder integratedAngleEncoder;
    private final AnalogEncoder angleEncoder;

    private final PIDController driveControllerSim;
    private final PIDController angleControllerSim;
    private final SparkPIDController angleController;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.kDrivetrain.DRIVE_KS, Constants.kDrivetrain.DRIVE_KV, Constants.kDrivetrain.DRIVE_KA);

    /* Drive Motor Control Requests */
    private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);

    /* Drive Motor Status Signals */
    private final StatusSignal<Double> drivePositionSignal;
    private final StatusSignal<Double> driveVelocitySignal;
    private final StatusSignal<Double> driveAppliedVoltsSignal;
    private final StatusSignal<Double> driveCurrentSignal;

    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        driveMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), Constants.kDrivetrain.DRIVE_GEAR_RATIO, 0.032); // This moment of inertia is a rough estimate for now
        driveControllerSim = new PIDController(Constants.kDrivetrain.DRIVE_KP, Constants.kDrivetrain.DRIVE_KI, Constants.kDrivetrain.DRIVE_KD);

        driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setPosition(0);

        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveAppliedVoltsSignal = driveMotor.getMotorVoltage();
        driveCurrentSignal = driveMotor.getSupplyCurrent();
        
        /* Angle Encoder Config */
        angleEncoder = new AnalogEncoder(moduleConstants.absoluteEncoderID);

        /* Angle Motor Config */
        angleMotor = SparkMaxFactory.createSparkMax(moduleConstants.angleMotorID, REVConfigs.angleSparkMaxConfig);
        angleMotorSim = new DCMotorSim(DCMotor.getNEO(1), Constants.kDrivetrain.ANGLE_GEAR_RATIO, 0.004); // This moment of inertia is a rough estimate for now
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        angleControllerSim = new PIDController(Constants.kDrivetrain.ANGLE_KP, Constants.kDrivetrain.ANGLE_KI, Constants.kDrivetrain.ANGLE_KD);

        integratedAngleEncoder.setPositionConversionFactor(Constants.kDrivetrain.ANGLE_POSITION_CONVERSION_FACTOR_DEGREES);
        angleController.setP(Constants.kDrivetrain.ANGLE_KP);
        angleController.setI(Constants.kDrivetrain.ANGLE_KI);
        angleController.setD(Constants.kDrivetrain.ANGLE_KD);
        angleController.setFF(Constants.kDrivetrain.ANGLE_KFF);
        resetToAbsolute();

        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.kDrivetrain.DEFAULT_DRIVE_FREQUENCY_HZ, 
            drivePositionSignal,
            driveVelocitySignal,
            driveAppliedVoltsSignal,
            driveCurrentSignal);
        
        lastAngle = getState().angle;
    }

    public void updateInputs() {
        BaseStatusSignal.refreshAll(
            drivePositionSignal,
            driveVelocitySignal,
            driveAppliedVoltsSignal,
            driveCurrentSignal
        );

        inputs.drivePositionMeters =
            Conversions.talonToMeters(drivePositionSignal.getValueAsDouble(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO);
        inputs.driveVelocityMetersPerSec =
            Conversions.talonToMPS(driveVelocitySignal.getValueAsDouble(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO);
        inputs.driveAppliedVolts = driveAppliedVoltsSignal.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrentSignal.getValueAsDouble();

        inputs.angleAbsolutePosition = Rotation2d.fromRotations(angleEncoder.getAbsolutePosition());
        inputs.anglePosition = Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        inputs.angleVelocityDegreesPerSec = integratedAngleEncoder.getVelocity();
        inputs.angleAppliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
        inputs.angleCurrentAmps = angleMotor.getOutputCurrent();        
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        
        targetState = desiredState;

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public void setPercentOutput(double drivePercentOutput, double anglePercentOutput) {
        driveDutyCycleRequest.Output = drivePercentOutput;
        driveMotor.setControl(driveDutyCycleRequest);
        angleMotor.set(anglePercentOutput);
    }

    public void setVolts(double driveVolts, double angleVolts) {
        driveVoltageRequest.Output = driveVolts;
        driveMotor.setControl(driveVoltageRequest);
        angleMotor.setVoltage(angleVolts);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (RobotBase.isReal()) {
            if (isOpenLoop) {
                driveDutyCycleRequest.Output = desiredState.speedMetersPerSecond / Constants.kDrivetrain.MAX_LINEAR_VELOCITY;
                driveMotor.setControl(driveDutyCycleRequest);
            }
            else {
                driveVelocityRequest.Velocity = Conversions.MPSToTalon(desiredState.speedMetersPerSecond, Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO);
                driveVelocityRequest.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
                driveMotor.setControl(driveVelocityRequest);
            }
        }
        else {
            driveMotorSim.setInputVoltage(driveFeedForward.calculate(desiredState.speedMetersPerSecond) + 
            driveControllerSim.calculate(Conversions.RPMToTalon(driveMotorSim.getAngularVelocityRPM(), Constants.kDrivetrain.DRIVE_GEAR_RATIO), Conversions.MPSToTalon(desiredState.speedMetersPerSecond, Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO)));
            driveMotorSim.update(0.02);
        }
    }
    

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.kDrivetrain.MAX_LINEAR_VELOCITY * 0.01))
                ? lastAngle
                : desiredState.angle;

        if (RobotBase.isReal()) {
            angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        }
        else {
            angleMotorSim.setInputVoltage(angleControllerSim.calculate(angleMotorSim.getAngularPositionRotations() * 360, desiredState.angle.getDegrees()));
            angleMotorSim.update(0.02);
        }
        lastAngle = angle;
    }

    private Rotation2d getIntegratedAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(angleEncoder.get());
    }

    public void resetToAbsolute() {
        integratedAngleEncoder.setPosition(getAbsoluteAngle().getDegrees() - angleOffset.getDegrees());
    }

    public void setDriveIdleMode(boolean setBrakeMode) {
        driveMotor.setNeutralMode(setBrakeMode? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setAngleIdleMode(boolean setBrakeMode) {
        angleMotor.setIdleMode(setBrakeMode? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void setDrivePID(double kP, double kI, double kD) {
        driveMotor.getConfigurator().apply(new Slot0Configs().
        withKP(kP).
        withKI(kI).
        withKD(kD));
    }

    public void setAnglePIDF(double kP, double kI, double kD, double kFF) {
        angleController.setP(kP);
        angleController.setI(kI);
        angleController.setD(kD);
        angleController.setFF(kFF);
    }

    public SwerveModuleState getState() {
        if (RobotBase.isReal()) {
            return new SwerveModuleState(
                Conversions.talonToMPS(inputs.driveVelocityMetersPerSec, Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO), 
                getIntegratedAngle()
            );
        }
        else {
            return new SwerveModuleState(
                Conversions.RPMToMPS(driveMotorSim.getAngularVelocityRPM(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE), 
                Rotation2d.fromRotations(angleMotorSim.getAngularPositionRotations()));
        }
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModulePosition getPosition() {
        if (RobotBase.isReal()) {
            return new SwerveModulePosition(
                Conversions.talonToMeters(inputs.drivePositionMeters, Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO), 
                getIntegratedAngle()
            );
        }
        else {
            return new SwerveModulePosition(driveMotorSim.getAngularPositionRotations() * Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, 
                Rotation2d.fromRotations(angleMotorSim.getAngularPositionRotations()));
        }
    }

    /**
     * @return The output current of the drive motor
     */
    public double getDriveOutputCurrent() {
       return driveMotor.getTorqueCurrent().getValueAsDouble();
    }
}