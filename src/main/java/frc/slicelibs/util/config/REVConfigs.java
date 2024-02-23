package frc.slicelibs.util.config;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;

public final class REVConfigs {

    public static class SparkMaxConfiguration {

        public IdleMode idleMode;
        public boolean inverted;

        public int statusFrame0PeriodMs = 10;
        public int statusFrame1PeriodMs;
        public int statusFrame2PeriodMs;

        public double openLoopRampRate;
        public double closedLoopRampRate;

        public boolean enableVoltageCompensation;
        public double nominalVoltage;

        public int currentLimit;    

        public SparkMaxConfiguration(
            IdleMode idleMode, 
            boolean inverted, 
            boolean enableVoltageCompensation, 
            double nominalVoltage, 
            double openLoopRampRate, 
            double closedLoopRampRate, 
            int currentLimit, 
            int statusFrame1RateMs,
            int statusFrame2RateMs) {

                this.idleMode = idleMode;
                this.inverted = inverted;
                this.enableVoltageCompensation = enableVoltageCompensation;
                this.nominalVoltage = nominalVoltage;
                this.openLoopRampRate = openLoopRampRate;
                this.closedLoopRampRate = closedLoopRampRate;
                this.currentLimit = currentLimit;
                this.statusFrame1PeriodMs = statusFrame1RateMs;
                this.statusFrame2PeriodMs = statusFrame2RateMs;

        }

    }

    public static final SparkMaxConfiguration defaultSparkMaxConfig = new SparkMaxConfiguration(
        IdleMode.kBrake,
        false,
        false,
        12,
        0.0,
        0.0,
        30,
        1000,
        1000);

    public static final SparkMaxConfiguration defaultVelocitySparkMaxConfig = new SparkMaxConfiguration(
        IdleMode.kBrake,
        false,
        false,
        12,
        0.0,
        0.0,
        30,
        200,
        1000);

    public static final SparkMaxConfiguration defaultPositionSparkMaxConfig = new SparkMaxConfiguration(
        IdleMode.kBrake,
        false,
        false,
        12,
        0.0,
        0.0,
        20,
        1500,
        300);

    public static final SparkMaxConfiguration angleSparkMaxConfig = new SparkMaxConfiguration(
        Constants.kDrivetrain.ANGLE_IDLE_MODE, 
        Constants.kDrivetrain.ANGLE_INVERT, 
        false, 
        12, 
        0.0, 
        0.0, 
        20, 
        Constants.kDrivetrain.ANGLE_FRAME_1_PERIOD_MS, 
        Constants.kDrivetrain.ANGLE_FRAME_2_PERIOD_MS);

    public static final SparkMaxConfiguration shooterSparkMaxConfig = new SparkMaxConfiguration(
        Constants.kShooter.IDLE_MODE, 
        Constants.kShooter.INVERT, 
        false, 
        12, 
        0.0, 
        0.0, 
        30, 
        Constants.kShooter.FRAME_1_PERIOD_MS, 
        Constants.kShooter.FRAME_2_PERIOD_MS);

    public static final SparkMaxConfiguration intakeSparkMaxConfig = new SparkMaxConfiguration(
        Constants.kIntake.ENTRANCE_IDLE_MODE, 
        Constants.kIntake.ENTRANCE_INVERT, 
        false, 
        12, 
        0.0, 
        0.0, 
        20, 
        Constants.kIntake.ENTRANCE_FRAME_1_PERIOD_MS, 
        Constants.kIntake.ENTRANCE_FRAME_2_PERIOD_MS);

}
