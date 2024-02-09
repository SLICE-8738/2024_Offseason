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
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kShooter;

/**
 * This subsystem controls the speed of the flywheel and the angle of the shooter. 
 */
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax flywheelTop, flywheelBottom; // Create the flywheel motor
  private CANSparkMax aimMotor; // Create the aiming motor
  private RelativeEncoder flyTopEncoder, flyBottomEncoder; // Encoder for both...
  private RelativeEncoder aimEncoder; // ...
  private SparkPIDController flyTopPID, flyBottomPID; // PIDs for both...
  private SparkPIDController aimPID; // ...
  private SimpleMotorFeedforward flyFeedforward;
  private double speedTarget; // Target speed of flywheel
  private double angleTarget; // Target angle of shooter

  private ShuffleboardTab shooterTesting;
  private SimpleWidget differential;

  public Shooter() {

    shooterTesting = Shuffleboard.getTab("Shooter Testing");
    differential = shooterTesting.add("Differential Testing: Top Flywheel", 0);

    // Define the above objects
    flywheelTop = new CANSparkMax(18, MotorType.kBrushless);
    flywheelBottom = new CANSparkMax(17, MotorType.kBrushless);

    aimMotor = new CANSparkMax(0, MotorType.kBrushless);
    flyTopEncoder = flywheelTop.getEncoder();
    flyBottomEncoder = flywheelBottom.getEncoder();
    aimEncoder = aimMotor.getEncoder();
    flyTopPID = flywheelTop.getPIDController();
    flyBottomPID = flywheelBottom.getPIDController();
    aimPID = aimMotor.getPIDController();
    flyFeedforward = new SimpleMotorFeedforward(0, kShooter.FLYWHEEL_FEED_FORWARD);
    
    // Set PID of flywheel and aim motors.
    flyTopPID.setP(kShooter.FLYWHEEL_KP);
    flyTopPID.setI(kShooter.FLYWHEEL_KI);
    flyTopPID.setD(kShooter.FLYWHEEL_KD);

    flyBottomPID.setP(kShooter.FLYWHEEL_KP);
    flyBottomPID.setI(kShooter.FLYWHEEL_KI);
    flyBottomPID.setD(kShooter.FLYWHEEL_KD);

    aimPID.setP(kShooter.AIM_KP);
    aimPID.setI(kShooter.AIM_KI);
    aimPID.setD(kShooter.AIM_KD);

    aimEncoder.setPosition(0);
  }

  /**
   * This function begins to spin up the fly wheel to a target speed.
   * @param speed Target speed.
   */
  public void spinFlywheel(double speed){
    double differentialMultiplier = differential.getEntry().getDouble(0);
    speedTarget = speed;
    flyTopPID.setReference(speed * (1+differentialMultiplier), ControlType.kVelocity, 0, flyFeedforward.calculate(speed)); // Spin up the flywheel to the target speed.
    flyBottomPID.setReference(speed * (1-differentialMultiplier), ControlType.kVelocity, 0, flyFeedforward.calculate(speed)); // Spin up the flywheel to the target speed.
  }

  /** 
   * Sets the speed of both flywheels to 0 and cancels any PID setpoints.
   */
  public void stopFlywheel() {

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
    aimPID.setReference(angle, ControlType.kPosition); // Move the shooter to the target angle.
  }

  /**
   * This function detects if the flywheel is at the target speed, within an acceptable error. If it is, the function returns true.
   * @param acceptableError The acceptable error that the flywheel's speed may be in.
   * @return True if at the correct speed, false otherwise.
   */
  public boolean atTargetSpeed(double acceptableError){
    double currentSpeed = flyTopEncoder.getVelocity(); // Get the current speed of the flywheel
    currentSpeed = (currentSpeed + flyBottomEncoder.getVelocity()) / 2; // Get the current speed of the other flywheel and average
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
    double currentAngle = aimEncoder.getPosition(); // Get the current angle of the shooter
    if (Math.abs(angleTarget - currentAngle) <= acceptableError){ // Is the current angle within the acceptable error?
      return true; // if so, true.
    }
    return false; // otherwise, false
  }

  public boolean isStowed(double acceptableError) {
    double currentAngle = aimEncoder.getPosition(); // Get the current angle of the shooter
    if (Math.abs(Constants.kShooter.SHOOTER_STOW_ANGLE - currentAngle) <= acceptableError){ // Is the current angle within the acceptable error?
      return true; // if so, true.
    }
    return false; // otherwise, false
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
