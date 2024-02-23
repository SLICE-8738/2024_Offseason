// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  public static final class kDrivetrain {

    public static final Port NAVX_PORT = Port.kUSB;
    public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.729);
    public static final double WHEEL_BASE = Units.inchesToMeters(18.299);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final double WHEEL_DIAMETER = 0.0935;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = (5.9 / 1.0); // 5.9:1
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
    public static final double DRIVE_KS = 0.0; //TODO: Possibly tune feedforward gains
    public static final double DRIVE_KV = 2.4103;
    public static final double DRIVE_KA = 0.0;

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
    //TODO: Find maximum velocities
    public static final double MAX_LINEAR_VELOCITY = 4.5; // meters per second
    public static final double MAX_ANGULAR_VELOCITY = 7; // radians per second

    /* PathPlanner Values */
    public static final double MAX_MODULE_VELOCITY = 3.5;

    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(3.5, 2.5, Math.PI * 2, Math.PI * 2);

    public static final double TRANSLATION_KP = 2.5;
    public static final double ROTATION_KP = 2.5;

    /* Speaker Alignment Values */
    public static final double kPSpeakerAlignRotation = 2.5;
    public static final double kISpeakerAlignRotation = 0;
    public static final double kDSpeakerAlignRotation = 0;

    /* Neutral Modes */
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;
    public static final NeutralModeValue DRIVE_IDLE_MODE = NeutralModeValue.Brake;

    /* Motor Inverts */
    public static final InvertedValue DRIVE_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final boolean ANGLE_INVERT = true;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int DRIVE_MOTOR_ID = 2;
      public static final int ANGLE_MOTOR_ID = 6;
      public static final int CANCODER_ID = 21;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(32.2);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }

    /* Back Left Module - Module 1 */
    public static final class Mod1 {
      public static final int DRIVE_MOTOR_ID = 1;
      public static final int ANGLE_MOTOR_ID = 5;
      public static final int CANCODER_ID = 22;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(136.4);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }

    /* Front Right Module - Module 2 */
    public static final class Mod2 {
      public static final int DRIVE_MOTOR_ID = 3;
      public static final int ANGLE_MOTOR_ID = 7;
      public static final int CANCODER_ID = 20;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(3.16);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int DRIVE_MOTOR_ID = 4;
      public static final int ANGLE_MOTOR_ID = 8;
      public static final int CANCODER_ID = 23;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(212.17);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }

  }

  public static final class kShooter {

    /* Idle Mode */
    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    /* Motor Invert */
    public static final boolean INVERT = false;

    /* Status Frame Periods */
    public static final int FRAME_1_PERIOD_MS = 500;
    public static final int FRAME_2_PERIOD_MS = 200;

    public static final double AIM_POSITION_CONVERSION_FACTOR = 360;
    public static final double AIM_VELOCITY_CONVERSION_FACTOR = AIM_POSITION_CONVERSION_FACTOR / 60;

    public static final double FLYWHEEL_KP = 0.1;
    public static final double FLYWHEEL_KI = 0;
    public static final double FLYWHEEL_KD = 0;
    public static final double FLYWHEEL_FEED_FORWARD = 0.002;

    public static final double AIM_KP = 0.1;
    public static final double AIM_KI = 0;
    public static final double AIM_KD = 0;

    // Flywheel Speed
    public static final double FLYWHEEL_RPM = 60;

    // Stow Angle for picking up game pieces
    public static final double SHOOTER_STOW_ANGLE = -2.3;

    // Shooter Measurements
    public static final double ANGLE_BETWEEN_FLYWHEELS = 20.153; // Angle between the flywheels from the pivot point of the shooter
    public static final double ANGLE_BETWEEN_HIGH_FLYWHEEL_AND_PIVOT = 118.719; // Angle between the high flywheel and pivot point from the low flywheel
    public static final double DISTANCE_TO_HIGHER_FLYWHEEL = 0.3556; // Center distance from pivot point to higher flywheel of shooter (in meters) (14")
    public static final double DISTANCE_TO_LOWER_FLYWHEEL = 0.2667; // Center distance from pivot point to lower flywheel of shooter (in meters) (10.5")

    // The constant you subtract the launch angle from to get the shooter angle
    public static final double LAUNCH_ANGLE_TO_SHOOTER_ANGLE = 180 - (270 - Constants.kShooter.ANGLE_BETWEEN_FLYWHEELS - Constants.kShooter.ANGLE_BETWEEN_HIGH_FLYWHEEL_AND_PIVOT);

    public static final double PIVOT_Y = 0.61595; // Hieght of the shooter pivot from "robot center"
    public static final double PIVOT_X = -0.072288; // Distance (front to back) of the shooter pivot from "robot center"

    public static final double FLYWHEEL_RPM_ACCEPTABLE_ERROR = 5; // The maximum error allowed in the flywheel RPM
    public static final double VERTICAL_AIM_ACCEPTABLE_ERROR = 2; // The maximum error allowed in the shooter angle vertically, in degrees
    public static final double HORIZONTAL_AIM_ACCEPTABLE_ERROR = 2; // The maximum error allowed in the shooter angle horizontally (controlled by drivetrain). in degrees
    public static final double MAXIMUM_SHOOTING_DRIVETRAIN_SPEED = 0.1; // The maximum speed that the drivetrain can move at and shoot
  }

  public static final class kIntake {

    /* Idle Mode */
    public static final IdleMode ENTRANCE_IDLE_MODE = IdleMode.kCoast;

    /* Motor Invert */
    public static final boolean ENTRANCE_INVERT = false;

    /* Status Frame Periods */
    public static final int ENTRANCE_FRAME_1_PERIOD_MS = 500;
    public static final int ENTRANCE_FRAME_2_PERIOD_MS = 200;

  }

  public static final class kElevator {
    public static final double CLIMB_HEIGHT = 0.5;
  }

  public static final class kFieldPositions {
    public static final Translation2d SPEAKER_POSITION = new Translation2d(0, 0);
    public static final Pose2d LEFT_STAGE_ALIGNMENT_POSITION = new Pose2d(0, 0, new Rotation2d());
    public static final Pose2d RIGHT_STAGE_ALIGNMENT_POSITION = new Pose2d(0, 0, new Rotation2d());
    public static final Pose2d CENTER_STAGE_ALIGNMENT_POSITION = new Pose2d(0, 0, new Rotation2d());
  }

}