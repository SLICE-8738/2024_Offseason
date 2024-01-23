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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter;

/**
 * This subsystem controls the speed of the flywheel and the angle of the shooter. 
 */
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax flywheel; // Create the flywheel motor
  private CANSparkMax aimMotor; // Create the aiming motor
  private RelativeEncoder flyEncoder; // Encoder for both...
  private RelativeEncoder aimEncoder; // ...
  private SparkPIDController flyPID; // PIDs for both...
  private SparkPIDController aimPID; // ...
  private double speedTarget; // Target speed of flywheel
  private double angleTarget; // Target angle of shooter

  public Shooter() {
    // Define the above objects
    flywheel = new CANSparkMax(0, MotorType.kBrushless);
    aimMotor = new CANSparkMax(0, MotorType.kBrushless);
    flyEncoder = flywheel.getEncoder();
    aimEncoder = aimMotor.getEncoder();
    flyPID = flywheel.getPIDController();
    aimPID = aimMotor.getPIDController();
    
    // Set PID of flywheel and aim motors.
    flyPID.setP(kShooter.PGainFly);
    flyPID.setI(kShooter.IGainFly);
    flyPID.setD(kShooter.DGainFly);

    aimPID.setP(kShooter.PGainAim);
    aimPID.setI(kShooter.IGainAim);
    aimPID.setD(kShooter.DGainAim);
  }

  /**
   * This function begins to spin up the fly wheel to a target speed.
   * @param speed Target speed.
   */
  public void spinFlywheel(double speed){
    speedTarget = speed;
    flyPID.setReference(speed, ControlType.kVelocity); // Spin up the flywheel to the target speed.
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
  public boolean detectFlywheelSpeed(double acceptableError){
    double currentSpeed = flyEncoder.getVelocity(); // Get the current speed of the flywheel
    if (Math.abs(speedTarget - currentSpeed) <= acceptableError){ // Is the current speed within the acceptable error?
      return true; // if so, true.
    }
    return false; // otherwise, false.
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
