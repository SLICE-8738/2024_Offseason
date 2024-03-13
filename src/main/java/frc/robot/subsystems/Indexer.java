// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.TimingBudget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.slicelibs.util.config.REVConfigs;
import frc.slicelibs.util.factories.SparkMaxFactory;

public class Indexer extends SubsystemBase {

  // Creates private variables
  private CANSparkMax highIndexMotor;
  private LaserCan laser;

  public Indexer() {
    highIndexMotor = SparkMaxFactory.createSparkMax(15, REVConfigs.indexerSparkMaxConfig); // creates new motor
    laser = new LaserCan(19); // creates new laserCan

    try {
      // configures settings for the laserCan
      laser.setRangingMode(RangingMode.SHORT); // sets ranging mode to short distance, which is more accurate
      laser.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS); // checks every 33 milliseconds for the measurement of the
                                                              // laser
      laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 12, 12)); // the area where the laserCan can sense
                                                                              // objects

    } catch (ConfigurationFailedException e) {
      // displays if the code doesn't work properly
      System.out.println("Error configuring laser CAN");
      System.out.println(e.getMessage());
    }
  }

  /**
   * Method that makes the high index motor spin depending on the isStored method
   */
  public void spinIndex(double speed) {
    highIndexMotor.set(speed); // sets motor speed
  }

  /** Method that checks if a note is at the high index motor */
  public boolean isStored() {
    // checks if the laserCan distance is more than 150 millimeters or less than 150
    // millimeters
    if (getLaserCanDistance() <= Constants.kIndexer.STORE_NOTE_TARGET + Constants.kIndexer.STORE_NOTE_ERROR_TOLERANCE && getLaserCanDistance() >= Constants.kIndexer.STORE_NOTE_TARGET - Constants.kIndexer.STORE_NOTE_ERROR_TOLERANCE) {
      // if the laserCAN distance is less than 150 millimeters, returns true and there
      // is a note stored in the high index motor
      return true;
    } else {
      // if the laserCAN distance is more than 150 millimeters, returns false and
      // there is no note at the high index motor
      return false;
    }
  }

  /**
   * 
   * @return
   */
  // Method that returns the distance from the laserCAN in millimeters
  public double getLaserCanDistance() {
    // returns the distance from the laserCAN in millimeters
    Measurement measurement = laser.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement.distance_mm;
    } else {
      return 1000;
    }
  }

  public double getOutputCurrent() {
    return highIndexMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LaserCAN Distance", getLaserCanDistance());

    SmartDashboard.putBoolean("Have Note", getLaserCanDistance() < 250);
  }
}
