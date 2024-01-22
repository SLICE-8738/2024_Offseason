// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeEntrance, intakeRamp;
  /** Creates a new Intake. */
  public Intake() {
    this.intakeEntrance = new CANSparkMax(0, MotorType.kBrushless);
    this.intakeRamp = new CANSparkMax(0, MotorType.kBrushless);
  }

  public void intakeOn(double speed) {
    intakeEntrance.set(speed);
    intakeRamp.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
