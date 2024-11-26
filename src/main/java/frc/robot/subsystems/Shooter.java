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

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.kShooter;
import frc.slicelibs.util.config.REVConfigs;
import frc.slicelibs.util.factories.SparkMaxFactory;
import frc.slicelibs.util.math.Conversions;

/**
 * This subsystem controls the speed of the flywheel and the angle of the shooter. 
 */
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public TalonFX flywheelTop, flywheelBottom; // Create the flywheel motors
  private CANSparkMax aimMotorLeft, aimMotorRight; // Create the aiming motors
  private RelativeEncoder aimRelativeEncoderLeft, aimRelativeEncoderRight; // ...
  private RelativeEncoder aimAlternateEncoder; // ...
  private PIDController aimPID;// ...
  private SimpleMotorFeedforward flyFeedforward;

  private DutyCycleOut flywheelDutyCycle = new DutyCycleOut(0);
  private VelocityVoltage flywheelVelocity = new VelocityVoltage(0);

  private double speedTarget; // Target speed of flywheel
  private double angleTarget; // Target angle of shooter


  private final ShuffleboardTab shooterTestTab;
  private final SimpleWidget currentAngleWidget;

  public boolean pidAimControl;

  public boolean shooterDisabled;

  public DigitalInput reedSwitchStow1, reedSwitchStow2, reedSwitchAmp1 , reedSwitchAmp2;

  public Shooter() {

    shooterTestTab = Shuffleboard.getTab("Shooter Testing");
    currentAngleWidget = shooterTestTab.add("Current Shooter Angle", 0);

    // Define the above objects
    flywheelTop = new TalonFX(Constants.kShooter.FLYWHEEL_TOP_ID);
    flywheelBottom = new TalonFX(Constants.kShooter.FLYWHEEL_BOTTOM_ID);
    aimMotorLeft = SparkMaxFactory.createSparkMax(12, REVConfigs.shooterAimSparkMaxConfig.withInvert(true));
    aimMotorRight = SparkMaxFactory.createSparkMax(9, REVConfigs.shooterAimSparkMaxConfig.withInvert(false));

    aimRelativeEncoderLeft = aimMotorLeft.getEncoder();
    aimRelativeEncoderRight = aimMotorRight.getEncoder();
    aimAlternateEncoder = aimMotorLeft.getAlternateEncoder(4096);
    aimAlternateEncoder.setInverted(true);
    aimPID = new PIDController(Constants.kShooter.AIM_KP, Constants.kShooter.AIM_KI, Constants.kShooter.AIM_KD);
    flyFeedforward = new SimpleMotorFeedforward(kShooter.FLYWHEEL_FF_KS, kShooter.FLYWHEEL_FF_KV);

    flywheelTop.getConfigurator().apply(Constants.CTRE_CONFIGS.flywheelFXConfig);
    flywheelBottom.getConfigurator().apply(Constants.CTRE_CONFIGS.flywheelFXConfig);

    aimRelativeEncoderLeft.setPositionConversionFactor(Constants.kShooter.AIM_POSITION_CONVERSION_FACTOR);
    aimRelativeEncoderLeft.setVelocityConversionFactor(Constants.kShooter.AIM_VELOCITY_CONVERSION_FACTOR);

    aimRelativeEncoderRight.setPositionConversionFactor(Constants.kShooter.AIM_POSITION_CONVERSION_FACTOR);
    aimRelativeEncoderRight.setVelocityConversionFactor(Constants.kShooter.AIM_VELOCITY_CONVERSION_FACTOR);

    aimRelativeEncoderLeft.setPosition(Constants.kShooter.SHOOTER_STOW_ANGLE);
    aimRelativeEncoderRight.setPosition(Constants.kShooter.SHOOTER_STOW_ANGLE);
    
    aimAlternateEncoder.setPositionConversionFactor(180);
    aimAlternateEncoder.setVelocityConversionFactor(3);
    aimAlternateEncoder.setPosition(Constants.kShooter.SHOOTER_STOW_ANGLE);

    pidAimControl = false;

    shooterDisabled = false;

    reedSwitchAmp1 = new DigitalInput(3);
    reedSwitchAmp2 = new DigitalInput(4);
    reedSwitchStow1 = new DigitalInput(2);
    reedSwitchStow2 = new DigitalInput(1);

  }

  /**
   * This function begins to spin up the fly wheels to a target speed.
   * @param speed Target speed.
   */
  public void spinFlywheels(double speed, boolean usePID){
    if (shooterDisabled) return;

    if (speed > 8000) {
      speed = 8000;
    }

    speedTarget = speed;

    if (usePID) {
      flywheelVelocity.Velocity = Conversions.RPMToTalon(speed, Constants.kShooter.FLYWHEEL_GEAR_RATIO);
    }
    flywheelVelocity.FeedForward = flyFeedforward.calculate(speed);
    flywheelTop.setControl(flywheelVelocity); // Spin up the flywheel to the target speed.
    flywheelBottom.setControl(flywheelVelocity); // Spin up the flywheel to the target speed.
  }

  /**
   * Spins the flywheel using normal duty cycle control instead of velocity PID
   * @param speed power to run the flywheel at from -1 to 1
   */
  public void dutyCycleSpinFlywheels(double speed) {
    if (shooterDisabled) return;

    flywheelDutyCycle.Output = speed;
    flywheelTop.setControl(flywheelDutyCycle);
    flywheelBottom.setControl(flywheelDutyCycle);
  }

  public void voltageSpinFlywheels(double volts) {
    if (shooterDisabled) return;

    flywheelTop.setVoltage(volts);
    flywheelBottom.setVoltage(volts);
  }

  
  /** 
   * Sets the speed of both flywheels to 0 and cancels any PID setpoints.
   */
  public void stopFlywheels() {
    dutyCycleSpinFlywheels(0);
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
    pidAimControl = false;
    if (speed > 0 && atAmp()) {
      aimMotorLeft.set(0);
      aimMotorRight.set(0);
      return;
    }
    aimMotorLeft.set(speed);
    aimMotorRight.set(speed);
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
    double currentSpeed = getFlywheelSpeed(); // Get the current speed of the flywheel
    if (speedTarget - currentSpeed <= acceptableError){ // Is the current speed within the acceptable error?
      return true; // if so, true.
    }
    return false; // otherwise, false.
  }

  /**
   * This function returns the current average speed of the flywheels;
   * @return The current average flywheel speed in RPM;
   */
  public double getFlywheelSpeed() {
    double topSpeed = Conversions.talonToRPM(flywheelTop.getVelocity().getValue(), Constants.kShooter.FLYWHEEL_GEAR_RATIO);
    double bottomSpeed = Conversions.talonToRPM(flywheelBottom.getVelocity().getValue(), Constants.kShooter.FLYWHEEL_GEAR_RATIO);
    return (topSpeed + bottomSpeed) / 2; // Get the current average speed of the flywheels
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
    return flywheelTop.getTorqueCurrent().getValueAsDouble();
  }
  public double getBottomOutputCurrent(){
    return flywheelBottom.getTorqueCurrent().getValueAsDouble();
  }
  public double getTopSpeed(){
    return Conversions.talonToRPM(flywheelTop.getVelocity().getValue(), Constants.kShooter.FLYWHEEL_GEAR_RATIO);
  }
  public double getBottomSpeed(){
    return Conversions.talonToRPM(flywheelBottom.getVelocity().getValue(), Constants.kShooter.FLYWHEEL_GEAR_RATIO);
  }

  public void slowDownAim() {
    aimPID.setP(Constants.kShooter.AIM_KP / 2);
  }

  public void resetAimSpeed() {
    aimPID.setP(Constants.kShooter.AIM_KP);
  }

  public boolean atAmp() {
    return !reedSwitchAmp2.get() || !reedSwitchAmp1.get();
  }

  public boolean atStow() {
    return (!reedSwitchStow1.get() || !reedSwitchStow2.get());
  }

  @Override
  public void periodic() {
    currentAngleWidget.getEntry().setDouble(getAlternateAngle());

    if (pidAimControl && !shooterDisabled) {
      double feedback = aimPID.calculate(getAlternateAngle(), angleTarget);
      if (feedback < 0 || !atAmp()) {
        aimMotorLeft.setVoltage(feedback);
        aimMotorRight.setVoltage(feedback);
      } else {
        aimPID.reset();
      }
    }

    SmartDashboard.putNumber("Flywheel Speed", getFlywheelSpeed());

    double alternatePosition = aimAlternateEncoder.getPosition();
    double relativePosition = (aimRelativeEncoderLeft.getPosition() + aimRelativeEncoderRight.getPosition()) / 2;

    double alternateVelocity = aimAlternateEncoder.getVelocity();
    double relativeVelocity = (aimRelativeEncoderLeft.getVelocity() + aimRelativeEncoderRight.getVelocity()) / 2;

    SmartDashboard.putNumber("Shooter Angle", alternatePosition);
    
    SmartDashboard.putBoolean("Amp Reed 1", !reedSwitchAmp1.get());
    SmartDashboard.putBoolean("Amp Reed 2", !reedSwitchAmp2.get());
    SmartDashboard.putBoolean("Stow Reed 1", !reedSwitchStow1.get());
    SmartDashboard.putBoolean("Stow Reed 2", !reedSwitchStow2.get());

  }
}
