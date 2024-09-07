/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import frc.robot.*;
import frc.robot.subsystems.ShooterLimelight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.util.PathPlannerLogging;

public class Drivetrain extends SubsystemBase {

  private final SwerveModule[] swerveMods;

  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private final SwerveDrivePoseEstimator m_odometry;
  private final AHRS m_gyro;
  public final Field2d m_field2d;
  private static final Lock odometryLock = new ReentrantLock();
  private static final CyclicBarrier threadBarrier = new CyclicBarrier(2);

  private Rotation2d fieldOrientedOffset;

  private Rotation2d simHeading = new Rotation2d();

  public double maxLinearVelocity = 4.5;
  public double maxAngularVelocity = 7;

  private final SysIdRoutine sysIdDriveRoutine;

  /** Creates a new Drivetrain. */
  public Drivetrain(SwerveModuleIO mod0IO, SwerveModuleIO mod1IO, SwerveModuleIO mod2IO, SwerveModuleIO mod3IO) {

    swerveMods = new SwerveModule[] {
      new SwerveModule(mod0IO, 0),
      new SwerveModule(mod1IO, 1),
      new SwerveModule(mod2IO, 2),
      new SwerveModule(mod3IO, 3)
    };

    OdometryThread.getInstance().start();

    m_gyro = new AHRS(Constants.kDrivetrain.NAVX_PORT);

    Timer.delay(1.0);
    resetModulesToAbsolute();
    resetHeading();

    m_field2d = new Field2d();

    // Create and push Field2d to SmartDashboard
    SmartDashboard.putData(m_field2d);

    m_odometry = new SwerveDrivePoseEstimator(
      Constants.kDrivetrain.kSwerveKinematics, 
      getHeading(), 
      getModulePositions(), 
      ShooterLimelight.getTable().getLastBotPoseBlue(),
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(0.1, 0.1, 0.1));

    fieldOrientedOffset = new Rotation2d();

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Display the trajector on the Field2d object
      m_field2d.getObject("Trajectory").setPoses(poses);
    });

    sysIdDriveRoutine = new SysIdRoutine(new Config(), new Mechanism(
      (volts) -> {
        for(SwerveModule mod : swerveMods) {
          mod.runCharacterization(volts.in(Units.Volts));
        }
      },
      null,
      this 
    ));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    lockOdometry();
    for (SwerveModule mod : swerveMods) {

      mod.updateInputs();

    }
    unlockOdometry();

    for (SwerveModule mod : swerveMods) {

      mod.periodic();

    }

    updateOdometry();

    m_field2d.setRobotPose(getPose());

    Logger.recordOutput("Field Position", getPose());
    Logger.recordOutput("Actual Module States", getModuleStates());
    Logger.recordOutput("Target Module States", getTargetModuleStates());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Sets the idle mode of all drive motors to either brake mode or coast mode.
   * 
   * @param enableBrakeMode Whether or not the idle mode of all
   *                        drive motors should be set to brake mode(false to set
   *                        to coast mode).
   * 
   */
  public void setDriveIdleMode(boolean enableBrakeMode) {

    for(SwerveModule mod : swerveMods) {

      mod.setDriveBrakeMode(enableBrakeMode);

    }

  }

  /**
   * Sets the idle mode of all angle motors to either brake mode or coast mode.
   * 
   * @param enableBrakeMode Whether or not the idle mode of all
   *                        angle motors should be set to brake mode(false to set
   *                        to coast mode).
   * 
   */
  public void setAngleIdleMode(boolean enableBrakeMode) {

    for(SwerveModule mod : swerveMods) {

      mod.setAngleBrakeMode(enableBrakeMode);

    }

  }

  public void setDrivePID(double kP, double kI, double kD) {

    /*for(SwerveModule mod : swerveMods) {

      mod.setDrivePID(kP, kI, kD);

    }*/

  }

  public void setAnglePIDF(double kP, double kI, double kD, double kFF) {

    /*for(SwerveModule mod : swerveMods) {

      mod.setAnglePIDF(kP, kI, kD, kFF);

    }*/

  }

  /**
   * Drives the robot at either given field-relative X, Y, and rotational
   * velocities or given
   * robot-relative forward, sideways, and rotational velocities.
   * 
   * <p>
   * If using robot-relative velocities, the X component of the Translation2d
   * object should be the forward velocity
   * and the Y component should be the sideways velocity.
   * 
   * @param transform       A Transform2d object representing either the desired
   *                        field-relative velocities in meters/second for the
   *                        robot to move at along the X and Y axes of the
   *                        field(forwards/backwards from driver POV), or the
   *                        desired robot-relative forward
   *                        and sideways velocities in meters/second for the robot
   *                        to move at, as well as the desired velocity in
   *                        radians/second for the
   *                        robot to rotate at.
   * 
   * @param isOpenLoop      Whether the accordingly generated states for the given
   *                        velocities should be set using open loop control for
   *                        the drive motors
   *                        of the swerve modules.
   * @param isFieldRelative Whether the given velocities are relative to the field
   *                        or not.
   */
  public void drive(Transform2d transform, boolean isOpenLoop, boolean isFieldRelative) {

    Rotation2d rotationWithOffset = getHeading().minus(fieldOrientedOffset);
    if (rotationWithOffset.getDegrees() > 360) {
      rotationWithOffset.minus(Rotation2d.fromDegrees(360));
    }
    if (rotationWithOffset.getDegrees() < 0) {
      rotationWithOffset.plus(Rotation2d.fromDegrees(360));
    }

    SwerveModuleState[] states = Constants.kDrivetrain.kSwerveKinematics.toSwerveModuleStates(
        isFieldRelative
            ? ChassisSpeeds.discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                  transform.getX(),
                  transform.getY(),
                  transform.getRotation().getRadians(),
                  rotationWithOffset),
                0.02)
            : ChassisSpeeds.discretize(
                new ChassisSpeeds(
                  transform.getX(), 
                  transform.getY(),
                  transform.getRotation().getRadians()),
                0.02));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.MAX_LINEAR_VELOCITY);

    setModuleStates(states, isOpenLoop);

  }

  /**
   * Updates the drivetrain odometry object to the robot's current position on the
   * field.
   * 
   * @return The new updated pose of the robot.
   */
  private void updateOdometry() {

    double[] sampleTimestamps = swerveMods[0].getOdometryTimestamps(); // All signals are sampled together
    
    for (int i = 0; i < sampleTimestamps.length; i++) {

      m_odometry.updateWithTime(sampleTimestamps[i], getHeading(), getOdometryPositions(i));

    }

    Pose2d visionPose = ShooterLimelight.getTable().getCurrentBotPoseBlue();

    if (visionPose != null && ShooterLimelight.getTable().getTargetDetected()) {
      
      if (ShooterLimelight.getTable().getTargetCameraSpacePose().getZ() <= 3.5 && !DriverStation.isAutonomousEnabled()) {

        m_odometry.addVisionMeasurement(new Pose2d(visionPose.getX(), visionPose.getY(), getPose().getRotation()), Timer.getFPGATimestamp());

      }
      
    }

  }

  /**
   * Allows resources being used by the thread in which
   * this is called to only be used in that thread. 
   */
  public static void lockOdometry() {

    odometryLock.lock();

  }

  /**
   * Allows resources being used by the thread in which
   * this is called to be used in any thread. 
   */
  public static void unlockOdometry() {

    odometryLock.unlock();

  }

  /**
   * Waits for the other of the two odometry threads to get to a
   * common barrier point.
   */
  public static void awaitThread() {

    try {
      threadBarrier.await();
    }
    catch (InterruptedException | BrokenBarrierException e) {
      Thread.currentThread().interrupt();
    }

  }

  /**
   * Returns the current pose of the robot without updating
   * the odometry.
   * 
   * @return The current estimated pose of the robot.
   */
  public Pose2d getPose() {

    return m_odometry.getEstimatedPosition();

  }

  /**
   * Returns the current pose of either the blue or red speaker
   * relative to the robot depending on the selected team station.
   * 
   * @return The current robot-relative pose of the speaker.
   */
  public Translation2d getSpeakerPosition() {

    Translation2d difference = DriverStation.getAlliance().get() == Alliance.Blue? 
    Constants.kFieldPositions.BLUE_SPEAKER_POSITION.minus(getPose().getTranslation())
    : Constants.kFieldPositions.RED_SPEAKER_POSITION.minus(getPose().getTranslation());

    return difference;

  }

  /**
   * @return The current distance of the robot from the primary in-view AprilTag
   */
  public double getAprilTagDistance() {

    return Constants.kFieldPositions.APRILTAG_POSITIONS[(int) ShooterLimelight.getTable().getAprilTagID() - 1].getDistance(getPose().getTranslation());

  }

  public boolean atAllianceWing() {
    Alliance alliance = DriverStation.getAlliance().get();
    double x = m_odometry.getEstimatedPosition().getX();

    if (alliance == Alliance.Blue) {
      return x < 5.87248;
    } else {
      return x > 16.54 - 5.87248;
    }
  }

  /**
   * Calculates and returns whether the robot is within
   * the tuned shooting range.
   * 
   * @return Whether the robot is in range.
   */
  public boolean inShootingRange() {

    Translation2d speaker = getSpeakerPosition();

    return Math.hypot(speaker.getX(), speaker.getY()) <= 3;

  }

  /**
   * Obtains and returns the current positions of all drivetrain swerve modules.
   * 
   * @return The current positions of all drivetrain swerve modules.
   */
  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(SwerveModule mod : swerveMods) {

      positions[mod.moduleNumber] = mod.getPosition();

    }

    return positions;

  }

  /**
   * Obtains and returns the current positions of all drivetrain swerve modules
   * sampled in the odometry thread at the given sample index.
   * 
   * @param sampleIndex
   * @return The current positions of all drivetrain swerve modules sampled
   *         in the odometry thread.
   */
  public SwerveModulePosition[] getOdometryPositions(int sampleIndex) {


    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : swerveMods) {

      positions[mod.moduleNumber] = mod.getOdometryPositions()[sampleIndex];

    }

    return positions;

  }

  /**
   * Obtains and returns the current states of all drivetrain swerve modules.
   * 
   * @return The current states of all drivetrain swerve modules.
   */
  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for(SwerveModule mod : swerveMods) {

      states[mod.moduleNumber] = mod.getState();

    }

    return states;

  }

  /**
   * Obtains and returns the target states that the drivetrain swerve modules have
   * been set to.
   * 
   * @return The target states that the drivetrain swerve modules have been set
   *         to.
   */
  public SwerveModuleState[] getTargetModuleStates() {

    SwerveModuleState[] targetStates = new SwerveModuleState[4];

    for(SwerveModule mod : swerveMods) {

      targetStates[mod.moduleNumber] = mod.getTargetState();

    }

    return targetStates;

  }

  /**
   * Obtains and returns the current absolute angle readings
   * in degrees from the CANCoders of all swerve modules without offsets.
   * 
   * @return The current absolute angle readings in degrees from the CANCoders
   *         of all swerve modules without offsets.
   */
  public double[] getCANCoderAngles() {

    double[] angles = new double[4];

    for(SwerveModule mod : swerveMods) {

      angles[mod.moduleNumber] = mod.getCANcoderAngle().getDegrees();

    }

    return angles;

  }

  /**
   * Sets the positions of the integrated angle motor
   * encoders of all swerve modules to the absolute position
   * readings of the CANCoders with their offsets being taken
   * into account.
   */
  public void resetModulesToAbsolute() {

    for(SwerveModule mod : swerveMods) {

      mod.resetToAbsolute();

    }

  }

  /**
   * Resets the position of the odometry object using a specified position.
   * 
   * @param position The desired position to reset the odometry of the robot to.
   */
  public void resetOdometry(Pose2d position) {

    m_odometry.resetPosition(getHeading(), getModulePositions(), position);

  }

  /** 
   * Resets the position of the odometry object using a specified rotation
   * while keeping the translation the same.
   * 
   * @param rotation The desired rotation to reset the odometry of the robot to.
   */
  public void resetRotation(Rotation2d rotation) {

    resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), rotation));

  }

  /**
   * Resets the rotation of the odometry object to the rotation received from
   * the shooter limelight.
   */
  public void resetToAprilTagRotation() {

    resetRotation(ShooterLimelight.getTable().getLastBotPoseBlue().getRotation());

  }

  public void resetFieldOrientedHeading() {
    fieldOrientedOffset = getHeading().minus(Rotation2d.fromDegrees(180));
    resetRotation(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue? 0 : 180));
  }

  public void reverseFieldOrientedHeading() {
    fieldOrientedOffset = getHeading();
    resetRotation(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue? 180 : 0));
  }

  /**
   * Obtains and returns the current heading of the robot as a Rotation2d from the
   * gyro object.
   * 
   * @return The current heading of the robot as a Rotation2d.
   */
  public Rotation2d getHeading() {

    if (RobotBase.isReal()) {
      return Rotation2d.fromDegrees(Constants.kDrivetrain.INVERT_GYRO? 
        MathUtil.inputModulus(-m_gyro.getYaw(), 0 , 360) 
          : MathUtil.inputModulus(m_gyro.getYaw(), 0, 360));
    }
    else {
      SwerveModulePosition[] modulePositions = getModulePositions();
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

      for (SwerveModule mod : swerveMods) {
        
        moduleDeltas[mod.moduleNumber] = 
          new SwerveModulePosition(
            modulePositions[mod.moduleNumber].distanceMeters -
              lastModulePositions[mod.moduleNumber].distanceMeters,
            modulePositions[mod.moduleNumber].angle
          );
        lastModulePositions[mod.moduleNumber] = modulePositions[mod.moduleNumber];

      }

      simHeading = simHeading.plus(new Rotation2d(Constants.kDrivetrain.kSwerveKinematics.toTwist2d(moduleDeltas).dtheta));
      return simHeading;
    }

  }

  /**
   * @return The current rotational velocity of the robot as a Rotation2d
   */
  public Rotation2d getRotationalVelocity() {

    return Rotation2d.fromDegrees(m_gyro.getRate());

  }

  public boolean facingDoubleSub() {
    double degrees = getPose().getRotation().getDegrees();
    return (degrees > 0 && degrees < 45) || (degrees > 315 && degrees < 360) || (degrees > 135 && degrees < 225);
  }

  /**
   * Obtains and returns the current pitch of the robot from -180 to 180 degrees from the gyro object.
   * 
   * @return The current pitch of the robot from -180 to 180 degrees.
   */
  public double getPitch() {

    return m_gyro.getPitch();

  }

  /**
   * Obtains and returns the current roll of the robot from -180 to 180 degrees,
   * with an offset of 1.7 degrees from the gyro object.
   * 
   * @return The current pitch of the robot from -180 to 180 degrees, with an
   *         offset of 1.7 degrees.
   */
  public double getRoll() {

    return m_gyro.getRoll();

  }

  /**
   * Resets the gyro yaw axis to a heading of 0.
   */
  public void resetHeading() {

    m_gyro.reset();

  }

  /**
   * Calculates and returns the current chassis speeds of the drivetrain using
   * the average forward and sideways velocities of the individual swerve modules
   * and the rotational velocity measured by the gyro.
   * 
   * @return The current chassis speeds of the drivetrain.
   */
  public ChassisSpeeds getChassisSpeeds() {

    return Constants.kDrivetrain.kSwerveKinematics.toChassisSpeeds(getModuleStates());

  }

  /**
   * Sets the desired states of all drivetrain motors to the given robot-relative chassis 
   * speeds after being converted to swerve module states.
   * 
   * @param speeds The desired chassis speeds to move the drivetrain at.
   * 
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {

    speeds.omegaRadiansPerSecond *= -1;
    setModuleStates(Constants.kDrivetrain.kSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02)), false);

  }

  /**
   * Sets the desired states of all drivetrain swerve modules to a specified
   * arrary of states using
   * closed loop control for the drive motors of the swerve modules.
   * 
   * @param states The desired states for all drivetrain swerve modules to be set
   *               to.
   * @param isOpenLoop Whether the accordingly generated states for the given
   *                   velocities should be set using open loop control for
   *                   the drive motors
   *                   of the swerve modules.
   */
  public void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.MAX_LINEAR_VELOCITY);

    for(SwerveModule mod : swerveMods) {

      mod.runSetpoint(states[mod.moduleNumber], isOpenLoop);

    }

  }

  /**
   * Runs the drive and angle motors of all swerve modules at the given duty
   * cycle percent outputs.
   * 
   * @param drivePercentOutput The percent output between -1 and 1 to set all
   *                           drive motors to.
   * @param anglePercentOutput The percent output between -1 and 1 to set all
   *                           angle motors to.
   */
  public void driveDutyCycle(double drivePercentOutput, double anglePercentOutput) {

    for(SwerveModule mod : swerveMods) {

      mod.runDutyCycle(drivePercentOutput, anglePercentOutput);

    }

  }

  public double[] getDriveOutputCurents(){

    double[] currents = new double[4];

    for (SwerveModule mod : swerveMods) {

      currents[mod.moduleNumber] = mod.getDriveOutputCurrent();

    }

    return currents;
  }

  public Command getSysIdDriveQuasistatic(Direction direction) {

    return sysIdDriveRoutine.quasistatic(direction).beforeStarting(SignalLogger::start).andThen(SignalLogger::stop);

  }

  public Command getSysIdDriveDynamic(Direction direction) {

    return sysIdDriveRoutine.dynamic(direction).beforeStarting(SignalLogger::start).andThen(SignalLogger::stop);

  }

} 