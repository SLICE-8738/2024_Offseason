package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.slicelibs.util.config.REVConfigs;
import frc.slicelibs.util.config.SwerveModuleConstants;
import frc.slicelibs.util.factories.SparkMaxFactory;
import frc.slicelibs.util.math.Conversions;
import frc.slicelibs.util.math.OnboardModuleState;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;
    private SwerveModuleState targetState = new SwerveModuleState();

    private CANSparkMax angleMotor;
    private TalonFX driveMotor;
    private DCMotorSim driveMotorSim;
    private DCMotorSim angleMotorSim;
    private RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;

    private final PIDController driveControllerSim;
    private final PIDController angleControllerSim;
    private final SparkPIDController angleController;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.kDrivetrain.DRIVE_KS, Constants.kDrivetrain.DRIVE_KV, Constants.kDrivetrain.DRIVE_KA);

    /* Drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VoltageOut driveVoltage = new VoltageOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        driveMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), Constants.kDrivetrain.DRIVE_GEAR_RATIO, 0.032); // This moment of inertia is a rough estimate for now
        driveControllerSim = new PIDController(Constants.kDrivetrain.DRIVE_KP, Constants.kDrivetrain.DRIVE_KI, Constants.kDrivetrain.DRIVE_KD);
        configDriveMotor();
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = SparkMaxFactory.createSparkMax(moduleConstants.angleMotorID, REVConfigs.angleSparkMaxConfig);
        angleMotorSim = new DCMotorSim(DCMotor.getNEO(1), Constants.kDrivetrain.ANGLE_GEAR_RATIO, 0.004); // This moment of inertia is a rough estimate for now
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        angleControllerSim = new PIDController(Constants.kDrivetrain.ANGLE_KP, Constants.kDrivetrain.ANGLE_KI, Constants.kDrivetrain.ANGLE_KD);
        configAngleMotor();
        
        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        
        targetState = desiredState;

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public void setPercentOutput(double drivePercentOutput, double anglePercentOutput) {
        driveDutyCycle.Output = drivePercentOutput;
        driveMotor.setControl(driveDutyCycle);
        angleMotor.set(anglePercentOutput);
    }

    public void setVolts(double driveVolts, double angleVolts) {
        driveVoltage.Output = driveVolts;
        driveMotor.setControl(driveVoltage);
        angleMotor.setVoltage(angleVolts);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (RobotBase.isReal()) {
            if (isOpenLoop) {
                driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.kDrivetrain.MAX_LINEAR_VELOCITY;
                driveMotor.setControl(driveDutyCycle);
            }
            else {
                driveVelocity.Velocity = Conversions.MPSToTalon(desiredState.speedMetersPerSecond, Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO);
                driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
                driveMotor.setControl(driveVelocity);
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

    public Rotation2d getCANcoderAngle() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    private Rotation2d waitForCANcoder() {
        /* Wait for up to 250ms for a new CANcoder position */
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().waitForUpdate(250).getValue());
    }

    public void resetToAbsolute() {
        double absolutePosition = waitForCANcoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder() {    
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
    }

    private void configAngleMotor() {
        integratedAngleEncoder.setPositionConversionFactor(Constants.kDrivetrain.ANGLE_POSITION_CONVERSION_FACTOR_DEGREES);
        angleController.setP(Constants.kDrivetrain.ANGLE_KP);
        angleController.setI(Constants.kDrivetrain.ANGLE_KI);
        angleController.setD(Constants.kDrivetrain.ANGLE_KD);
        angleController.setFF(Constants.kDrivetrain.ANGLE_KFF);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0);
        driveMotor.getVelocity().setUpdateFrequency(Constants.kDrivetrain.DRIVE_VELOCITY_FRAME_RATE_HZ);
        driveMotor.getPosition().setUpdateFrequency(Constants.kDrivetrain.DRIVE_POSITION_FRAME_RATE_HZ);
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
                Conversions.talonToMPS(driveMotor.getVelocity().getValue(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO), 
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
                Conversions.talonToMeters(driveMotor.getPosition().getValue(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO), 
                getIntegratedAngle()
            );
        }
        else {
            return new SwerveModulePosition(driveMotorSim.getAngularPositionRotations() * Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, 
                Rotation2d.fromRotations(angleMotorSim.getAngularPositionRotations()));
        }
    }

    //returns the output current of driveMotor 
    public double getDriveOutputCurrent() {
       return driveMotor.getTorqueCurrent().getValueAsDouble();
    }
}