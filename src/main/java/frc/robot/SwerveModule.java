package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.slicelibs.util.config.CTREConfigs;
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
    private TalonFXSimState driveMotorSim;
    private RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;

    private final CTREConfigs ctreConfigs = new CTREConfigs();

    private final SparkPIDController angleController;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.kDrivetrain.DRIVE_KS, Constants.kDrivetrain.DRIVE_KV, Constants.kDrivetrain.DRIVE_KA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final VoltageOut driveVoltage = new VoltageOut(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = SparkMaxFactory.createSparkMax(moduleConstants.angleMotorID, REVConfigs.angleSparkMaxConfig);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        driveMotorSim = new TalonFXSimState(driveMotor);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
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

    public void setVoltage(double driveVolts, double angleVolts) {
        driveVoltage.Output = driveVolts;
        driveMotor.setControl(driveVoltage);
        angleMotor.setVoltage(angleVolts);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.kDrivetrain.MAX_LINEAR_VELOCITY;
            driveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToTalon(desiredState.speedMetersPerSecond, Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }
    

    private void setAngle(SwerveModuleState desiredState){
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.kDrivetrain.MAX_LINEAR_VELOCITY * 0.01))
                ? lastAngle
                : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    private Rotation2d waitForCANcoder(){
        /* wait for up to 250ms for a new CANcoder position */
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().waitForUpdate(250).getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = waitForCANcoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){    
        angleEncoder.getConfigurator().apply(ctreConfigs.swerveCANcoderConfig);
    }

    private void configAngleMotor(){
        integratedAngleEncoder.setPositionConversionFactor(Constants.kDrivetrain.ANGLE_POSITION_CONVERSION_FACTOR_DEGREES);
        angleController.setP(Constants.kDrivetrain.ANGLE_KP);
        angleController.setI(Constants.kDrivetrain.ANGLE_KI);
        angleController.setD(Constants.kDrivetrain.ANGLE_KD);
        angleController.setFF(Constants.kDrivetrain.ANGLE_KFF);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){
        driveMotor.getConfigurator().apply(ctreConfigs.swerveDriveFXConfig);
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

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.talonToMPS(driveMotor.getVelocity().getValue(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO), 
            getAngle()
        ); 
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.talonToMeters(driveMotor.getPosition().getValue(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }

    public void setSimulationPosition() {
        driveMotorSim.addRotorPosition(Conversions.metersToTalon(targetState.speedMetersPerSecond * 0.02, Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO));
        integratedAngleEncoder.setPosition(lastAngle.getDegrees());
    }

    public SparkPIDController getAngleController() {

        return angleController;

    }
}