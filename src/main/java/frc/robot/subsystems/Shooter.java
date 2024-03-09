// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Goals
//
// Spin flywheel motors at a specific velocity
// Aim shooter at specific position

// FUNCTIONS
// Spin up flywheel
// Detect flywheel speed
// Aim shooter
// Detect if shooter is in correct position

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.kShooter;
import frc.slicelibs.util.config.REVConfigs;
import frc.slicelibs.util.factories.SparkMaxFactory;

/**
 * This subsystem controls the speed of the flywheel and the angle of the shooter. 
 */
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax flywheelTop, flywheelBottom; // Create the flywheel motors
  private CANSparkMax aimMotorLeft, aimMotorRight; // Create the aiming motors
  private RelativeEncoder flyTopEncoder, flyBottomEncoder; // Encoder for both...
  private RelativeEncoder aimRelativeEncoderLeft, aimRelativeEncoderRight; // ...
  private RelativeEncoder aimAlternateEncoder; // ...
  private SparkPIDController flyTopPID, flyBottomPID; // PIDs for both...
  private PIDController aimPID;// ...
  private SimpleMotorFeedforward flyFeedforward;
  private double speedTarget; // Target speed of flywheel
  private double angleTarget; // Target angle of shooter

  private ShuffleboardTab shooterTesting;
  private SimpleWidget differential;

  private final ShuffleboardTab shooterTestTab;
  private final SimpleWidget currentAngleWidget;

  public boolean pidAimControl;

  public boolean shooterDisabled;

  public Shooter() {

    shooterTestTab = Shuffleboard.getTab("Shooter Testing");
    currentAngleWidget = shooterTestTab.add("Current Shooter Angle", 0);

    shooterTesting = Shuffleboard.getTab("Shooter Testing");
    differential = shooterTesting.add("Differential Testing: Top Flywheel", 0);

    // Define the above objects
    flywheelTop = SparkMaxFactory.createSparkMax(10, REVConfigs.shooterFlywheelSparkMaxConfig);
    flywheelBottom = SparkMaxFactory.createSparkMax(11, REVConfigs.shooterFlywheelSparkMaxConfig);
    aimMotorLeft = SparkMaxFactory.createSparkMax(12, REVConfigs.shooterAimSparkMaxConfig.withInvert(true));
    aimMotorRight = SparkMaxFactory.createSparkMax(9, REVConfigs.shooterAimSparkMaxConfig.withInvert(false));

    flyTopEncoder = flywheelTop.getEncoder();
    flyBottomEncoder = flywheelBottom.getEncoder();
    aimRelativeEncoderLeft = aimMotorLeft.getEncoder();
    aimRelativeEncoderRight = aimMotorRight.getEncoder();
    aimAlternateEncoder = aimMotorLeft.getAlternateEncoder(4096);
    aimAlternateEncoder.setInverted(true);
    flyTopPID = flywheelTop.getPIDController();
    flyBottomPID = flywheelBottom.getPIDController();
    aimPID = new PIDController(Constants.kShooter.AIM_KP, Constants.kShooter.AIM_KI, Constants.kShooter.AIM_KD);
    flyFeedforward = new SimpleMotorFeedforward(0, kShooter.FLYWHEEL_FEED_FORWARD);

    aimRelativeEncoderLeft.setPositionConversionFactor(Constants.kShooter.AIM_POSITION_CONVERSION_FACTOR);
    aimRelativeEncoderLeft.setVelocityConversionFactor(Constants.kShooter.AIM_VELOCITY_CONVERSION_FACTOR);

    aimRelativeEncoderRight.setPositionConversionFactor(Constants.kShooter.AIM_POSITION_CONVERSION_FACTOR);
    aimRelativeEncoderRight.setVelocityConversionFactor(Constants.kShooter.AIM_VELOCITY_CONVERSION_FACTOR);

    aimRelativeEncoderLeft.setPosition(Constants.kShooter.SHOOTER_STOW_ANGLE);
    aimRelativeEncoderRight.setPosition(Constants.kShooter.SHOOTER_STOW_ANGLE);
    
    aimAlternateEncoder.setPositionConversionFactor(180);
    aimAlternateEncoder.setVelocityConversionFactor(3);
    aimAlternateEncoder.setPosition(Constants.kShooter.SHOOTER_STOW_ANGLE);

    // Set PID of flywheel and aim motors.
    flyTopPID.setP(kShooter.FLYWHEEL_KP);
    flyTopPID.setI(kShooter.FLYWHEEL_KI);
    flyTopPID.setD(kShooter.FLYWHEEL_KD);

    flyBottomPID.setP(kShooter.FLYWHEEL_KP);
    flyBottomPID.setI(kShooter.FLYWHEEL_KI);
    flyBottomPID.setD(kShooter.FLYWHEEL_KD);

    pidAimControl = false;

    shooterDisabled = false;

  }

  /**
   * This function begins to spin up the fly wheels to a target speed.
   * @param speed Target speed.
   */
  public void spinFlywheels(double speed){
    double differentialMultiplier = 0;
    speedTarget = speed;
    flyTopPID.setReference(speed * (1 + differentialMultiplier), ControlType.kVelocity, 0, flyFeedforward.calculate(speed)); // Spin up the flywheel to the target speed.
    flyBottomPID.setReference(speed * (1 - differentialMultiplier), ControlType.kVelocity, 0, flyFeedforward.calculate(speed)); // Spin up the flywheel to the target speed.
  }

  /** 
   * Sets the speed of both flywheels to 0 and cancels any PID setpoints.
   */
  public void stopFlywheels() {
    flywheelTop.set(0);
    flywheelBottom.set(0);
  }

  /**
   * Spins the flywheel using normal duty cycle control instead of velocity PID
   * @param speed power to run the flywheel at from -1 to 1
   */
  public void dutyCycleSpinFlywheel(double speed) {
    flywheelTop.set(-speed);
    flywheelBottom.set(-speed);
  }

  /**
   * This function begins to move the shooter to a target angle.
   * @param angle Target angle.
   */
  public void aimShooter(double angle){
    angleTarget = angle;
    
    // Move the shooter to the target angle 
    pidAimControl = true;
  }

  /**
   * This function runs the pivot motors at the given percent output.
   * @param speed Power to run the aim motors at from -1 to 1
   */
  public void dutyCycleAimShooter(double speed) {
    aimMotorLeft.set(speed);
    aimMotorRight.set(speed);

    pidAimControl = false;
  }

  /**
   * @return The current absolute angle of the shooter pivot in degrees with no bounds
   *         read from the alternate encoder.
   */
  public double getAlternateAngle() {
    return aimAlternateEncoder.getPosition();
  }

  /**
   * @return The current relative angle of the shooter pivot in degrees with no bounds
   *         read from the relative encoders.
   */
  public double getRelativeAngle() {
    return (aimRelativeEncoderLeft.getPosition() + aimRelativeEncoderRight.getPosition()) / 2;
  }

  public void setAlternateAngle(double angle) {
    aimAlternateEncoder.setPosition(angle);
  }

  /**
   * This function detects if the flywheel is at the target speed, within an acceptable error. If it is, the function returns true.
   * @param acceptableError The acceptable error that the flywheel's speed may be in.
   * @return True if at the correct speed, false otherwise.
   */
  public boolean atTargetSpeed(double acceptableError){
    double currentSpeed = flyTopEncoder.getVelocity(); // Get the current speed of the flywheel
    currentSpeed = (currentSpeed + flyBottomEncoder.getVelocity()) / 2; // Get the current speed of the other flywheel and average
    SmartDashboard.putNumber("Flywheel Speed Error", speedTarget - currentSpeed);
    if (Math.abs(speedTarget - currentSpeed) <= acceptableError){ // Is the current speed within the acceptable error?
      return true; // if so, true.
    }
    return false; // otherwise, false.
  }

  public double getFlywheelSpeed() {
    double currentSpeed = flyTopEncoder.getVelocity(); // Get the current speed of the flywheel
    currentSpeed = (currentSpeed + flyBottomEncoder.getVelocity()) / 2; // Get the current speed of the other flywheel and average
    return currentSpeed;
  }


  /**
   * This function detects if the shooter is at the target angle, within an acceptable error. If it is, the function returns true.
   * @param acceptableError The acceptable error that the shooter's angle may be in.
   * @return True if at the correct angle, false otherwise.
   */
  public boolean detectShooterAngle(double acceptableError){
    double currentAngle = getAlternateAngle(); // Get the current angle of the shooter
    if (Math.abs(angleTarget - currentAngle) <= acceptableError){ // Is the current angle within the acceptable error?
      return true; // if so, true.
    }
    return false; // otherwise, false
  }

  public boolean isStowed(double acceptableError) {
    double currentAngle = getAlternateAngle(); // Get the current angle of the shooter
    if (Math.abs(Constants.kShooter.SHOOTER_STOW_ANGLE - currentAngle) <= acceptableError){ // Is the current angle within the acceptable error?
      return true; // if so, true.
    }
    return false; // otherwise, false
  }

  public double getTopOutputCurrent(){
    return flywheelTop.getOutputCurrent();
  }
  public double getBottomOutputCurrent(){
    return flywheelBottom.getOutputCurrent();
  }

  public void slowDownAim() {
    aimPID.setP(Constants.kShooter.AIM_KP / 2);
  }

  public void resetAimSpeed() {
    aimPID.setP(Constants.kShooter.AIM_KP);
  }

  @Override
  public void periodic() {

    currentAngleWidget.getEntry().setDouble(getAlternateAngle());

    if (pidAimControl) {
      double feedback = aimPID.calculate(getAlternateAngle(), angleTarget);
      aimMotorLeft.setVoltage(feedback);
      aimMotorRight.setVoltage(feedback);
    }

    SmartDashboard.putNumber("Flywheel Speed", getFlywheelSpeed());

    double alternatePosition = aimAlternateEncoder.getPosition();
    double relativePosition = (aimRelativeEncoderLeft.getPosition() + aimRelativeEncoderRight.getPosition()) / 2; 
    if (Math.abs(alternatePosition - relativePosition) > 15) {
      shooterDisabled = true;
    }

  }
}
