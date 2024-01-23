// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;

import frc.slicelibs.util.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;

    public static final double driveExponent = 1.0;
    public static final double driveExponentPercent = 1;

    public static final double turnExponent = 1.0;
    public static final double turnExponentPercent = 1;

  }

  public static final class kShooter {
    public static final double PGainFly = 0.1;
    public static final double IGainFly = 0;
    public static final double DGainFly = 0;
    public static final double PGainAim = 0.1;
    public static final double IGainAim = 0;
    public static final double DGainAim = 0;
  }

  public static final class kElevator {
    public static final double CLIMB_HEIGHT = 0.5;
  }

  public static final class kDrivetrain {

    public static final Port NAVX_PORT = Port.kUSB;
    public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW- (except for some godforsaken
                                                     // reason CCW- CW+ is the one that actually works so ignore this)

    /* Drivetrain Constants */
    //TODO: Measure chassis and wheel dimensions
    public static final double TRACK_WIDTH = Units.inchesToMeters(22);
    public static final double WHEEL_BASE = Units.inchesToMeters(27);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final double WHEEL_DIAMETER = 0.0935;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0); // 6.75:1
    public static final double ANGLE_GEAR_RATIO = ((150.0 / 7.0) / 1.0); // (150/7):1

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /* Swerve Voltage Compensation */
    public static final double MAX_VOLTAGE = 12.0;

    /* Swerve Current Limiting */
    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int DRIVE_CURRENT_THRESHOLD = 60;
    public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* Status Frame Rates/Periods */
    //TODO: Tune status frames
    public static final int DRIVE_VELOCITY_FRAME_RATE_HZ = 22;
    public static final int DRIVE_POSITION_FRAME_RATE_HZ = 5;
    public static final int ANGLE_FRAME_1_PERIOD_MS = 1500;
    public static final int ANGLE_FRAME_2_PERIOD_MS = 300;

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = 0.01;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.001;
    public static final double ANGLE_KFF = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.12; //TODO: Tune drive motor PID gains
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double DRIVE_KS = 0.23065; //TODO: Possibly tune feedforward gains
    public static final double DRIVE_KV = 2.717;
    public static final double DRIVE_KA = 0.32115;

    /* Drive Motor Conversion Factors */
    public static final double DRIVE_POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR / 60.0;
    public static final double ANGLE_POSITION_CONVERSION_FACTOR_DEGREES = 360.0 / ANGLE_GEAR_RATIO;
    public static final double ANGLE_POSITION_CONVERSION_FACTOR_RADIANS = Math.PI * 2;
    public static final double ANGLE_VELOCITY_CONVERSION_FACTOR_DEGREES = ANGLE_POSITION_CONVERSION_FACTOR_DEGREES
        / 60.0;
    public static final double ANGLE_VELOCITY_CONVERSION_FACTOR_RADIANS = ANGLE_POSITION_CONVERSION_FACTOR_RADIANS
        / 60.0;

    /* Swerve Profiling Values */
    public static final double MAX_LINEAR_VELOCITY = 4.5; // meters per second
    public static final double MAX_ANGULAR_VELOCITY = 7; // radians per second

    /* Neutral Modes */
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;
    public static final NeutralModeValue DRIVE_IDLE_MODE = NeutralModeValue.Brake;

    /* Motor Inverts */
    public static final InvertedValue DRIVE_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final boolean ANGLE_INVERT = true;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;
    // Charge Station Board and Balance Speeds (-1 - 1)
    // public static final double CHARGE_STATION_BALANCE_SPEED = 0.3;
    // public static final double BOARD_CHARGE_SPEED = -0.55;

    /*
     * Charge Station Board and Balance Speeds (meters/second)
     * THESE SPEEDS WERE CALCULATED BASED ON THE CURRENT MAXIMUM VELOCITY
     * CONSTANT AND ARE PLACEHOLDERS FOR NOW
     */
    public static final double CHARGE_STATION_BALANCE_SPEED = 1.35;
    public static final double BOARD_CHARGE_SPEED = 2.475;

    // The minimum angle the drivetrain must be at to stop when boarding the charge
    // station
    public static final double BOARD_CHARGE_MINIMUM_STOP_ANGLE = 6;
    // The amount the angle should drop below the maximum angle to stop boarding the
    // charge station and begin balancing
    public static final double BOARD_CHARGE_ANGLE_CHANGE_THRESHOLD = 3.8;

    // Autnomous Tolerances
    public static final double AUTO_DISTANCE_ERROR_TOLERANCE = 0.35;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    //TODO: Find all module angle offsets and device IDs
    public static final class Mod0 {
      public static final int DRIVE_MOTOR_ID = 11;
      public static final int ANGLE_MOTOR_ID = 15;
      public static final int CANCODER_ID = 21;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(208.74);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }

    /* Back Left Module - Module 1 */
    public static final class Mod1 {
      public static final int DRIVE_MOTOR_ID = 16;
      public static final int ANGLE_MOTOR_ID = 12;
      public static final int CANCODER_ID = 20;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(311.4);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }

    /* Front Right Module - Module 2 */
    public static final class Mod2 {
      public static final int DRIVE_MOTOR_ID = 17;
      public static final int ANGLE_MOTOR_ID = 13;
      public static final int CANCODER_ID = 22;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(247.94);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int DRIVE_MOTOR_ID = 14;
      public static final int ANGLE_MOTOR_ID = 18;
      public static final int CANCODER_ID = 23;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(32.08);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }
  }

  public static final class kAutonomous {

    public static final double kMaxVelocityMetersPerSecond = 3.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
    public static final double kMaxAngularVelocityRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    public static final PathConstraints kPathConstraints = new PathConstraints(3.5, 2.5, Math.PI, Math.PI);

    public static final double kPTranslation = 2.5;
    public static final double kPRotation = 2.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularVelocityRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

  }
}
