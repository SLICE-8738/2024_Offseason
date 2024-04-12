// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.slicelibs.util.config.REVConfigs;
import frc.slicelibs.util.factories.SparkMaxFactory;
/**
 * An intake pulls in the ring (or "note") that will later be launched.
 */
public class Intake extends SubsystemBase {
  public CANSparkMax intakeEntrance, intakeRamp;
  public RelativeEncoder intakeEntranceEncoder, intakeRampEncoder; 
  /** Creates a new Intake. */
  public Intake() {
    this.intakeEntrance = SparkMaxFactory.createSparkMax(13, REVConfigs.intakeEntranceSparkMaxConfig);
    this.intakeRamp = SparkMaxFactory.createSparkMax(14, REVConfigs.intakeRampSparkMaxConfig);
    this.intakeEntranceEncoder = intakeEntrance.getEncoder();
    this.intakeRampEncoder = intakeRamp.getEncoder();
  }

  /**
   * Sets the spinning speed of the intake motors.
   * @param speed
   */
  public void runIntake(double speed) {
    intakeEntrance.set(speed);
    if(getEntranceOutputCurrent() < Constants.kIntake.ENTRANCE_CURRENT_THRESHOLD) {
      intakeRamp.set(speed * 1.5);
    }
  }

  /**
   * Sets the spinning speed of only the upper ramp intake motor
   * @param speed
   */
  public void runRampIntakeOnly(double speed) {
    intakeRamp.set(speed);
  }

  public void runIntakeEntranceOnly(double speed) {
    intakeEntrance.set(speed);
  }

  public double getEntranceOutputCurrent() {
    return intakeEntrance.getOutputCurrent();
  }

  public double getRampOutputCurrent() {
    return intakeRamp.getOutputCurrent();
  }

  public double getEntranceVelocity(){
    return intakeEntranceEncoder.getVelocity();
  }

  public double getRampVelocity(){
    return intakeRampEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
