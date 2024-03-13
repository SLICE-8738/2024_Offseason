package frc.slicelibs.util.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public TalonFXConfiguration flywheelFXConfig = new TalonFXConfiguration();

    public CTREConfigs(){
        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        var driveMotorOutput = swerveDriveFXConfig.MotorOutput;
        driveMotorOutput.Inverted = Constants.kDrivetrain.DRIVE_INVERT;
        driveMotorOutput.NeutralMode = Constants.kDrivetrain.DRIVE_IDLE_MODE;

        /* Current Limiting */
        var driveCurrentLimits = swerveDriveFXConfig.CurrentLimits;
        driveCurrentLimits.SupplyCurrentLimitEnable = Constants.kDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT;
        driveCurrentLimits.SupplyCurrentLimit = Constants.kDrivetrain.DRIVE_CURRENT_LIMIT;
        driveCurrentLimits.SupplyCurrentThreshold = Constants.kDrivetrain.DRIVE_CURRENT_THRESHOLD;
        driveCurrentLimits.SupplyTimeThreshold = Constants.kDrivetrain.DRIVE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        var driveSlot0 = swerveDriveFXConfig.Slot0;
        driveSlot0.kP = Constants.kDrivetrain.DRIVE_KP;
        driveSlot0.kI = Constants.kDrivetrain.DRIVE_KI;
        driveSlot0.kD = Constants.kDrivetrain.DRIVE_KD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.kDrivetrain.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.kDrivetrain.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.kDrivetrain.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kDrivetrain.CLOSED_LOOP_RAMP;

        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.kDrivetrain.CANCODER_INVERT;
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        /** Flywheel Motor Configuration */
        var flywheelMotorOutput = flywheelFXConfig.MotorOutput;
        flywheelMotorOutput.Inverted = Constants.kShooter.FLYWHEEL_INVERT;
        flywheelMotorOutput.NeutralMode = Constants.kShooter.FLYWHEEL_IDLE_MODE;

        /* Current Limiting */
        var flywheelCurrentLimits = flywheelFXConfig.CurrentLimits;
        flywheelCurrentLimits.SupplyCurrentLimitEnable = Constants.kShooter.FLYWHEEL_ENABLE_CURRENT_LIMIT;
        flywheelCurrentLimits.SupplyCurrentLimit = Constants.kShooter.FLYWHEEL_CURRENT_LIMIT;
        flywheelCurrentLimits.SupplyCurrentThreshold = Constants.kShooter.FLYWHEEL_CURRENT_THRESHOLD;
        flywheelCurrentLimits.SupplyTimeThreshold = Constants.kShooter.FLYWHEEL_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        var flywheelSlot0 = flywheelFXConfig.Slot0;
        flywheelSlot0.kP = Constants.kShooter.FLYWHEEL_KP;
        flywheelSlot0.kI = Constants.kShooter.FLYWHEEL_KI;
        flywheelSlot0.kD = Constants.kShooter.FLYWHEEL_KD;

        /* Open and Closed Loop Ramping */
        flywheelFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.kShooter.FLYWHEEL_OPEN_LOOP_RAMP;
        flywheelFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.kShooter.FLYWHEEL_OPEN_LOOP_RAMP;

        flywheelFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.kShooter.FLYWHEEL_CLOSED_LOOP_RAMP;
        flywheelFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kShooter.FLYWHEEL_CLOSED_LOOP_RAMP;
    }
}