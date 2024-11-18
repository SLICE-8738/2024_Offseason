// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

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
  private RelativeEncoder encoder;
  private LaserCan lowLaser, highLaser;
  private boolean lowLaserOnline, highLaserOnline;

  public Indexer() {
    highIndexMotor = SparkMaxFactory.createSparkMax(15, REVConfigs.indexerSparkMaxConfig); // creates new motor

    encoder = highIndexMotor.getEncoder();
    lowLaser = new LaserCan(20); // creates new laserCan
    highLaser = new LaserCan(19);

    lowLaserOnline = true;
    highLaserOnline = true;

    try {
      // configures settings for the laserCan
      lowLaser.setRangingMode(RangingMode.SHORT); // sets ranging mode to short distance, which is more accurate
      lowLaser.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS); // checks every 33 milliseconds for the measurement of the laser
      lowLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 12, 12)); // the area where the laserCan can sense objects
      highLaser.setRangingMode(RangingMode.SHORT); // sets ranging mode to short distance, which is more accurate
      highLaser.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS); // checks every 33 milliseconds for the measurement of the laser
      highLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 12, 12)); // the area where the laserCan can sense objects

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
    return getLowLaserCanDistance() < Constants.kIndexer.DEFAULT_LASERCAN_DISTANCE;
  }

  /**
   * 
   * @return
   */
  // Method that returns the distance from the laserCAN in millimeters
  public double getLowLaserCanDistance() {
    // returns the distance from the laserCAN in millimeters
    Measurement measurement = lowLaser.getMeasurement();
    lowLaserOnline = measurement != null;
    return getDistance(measurement);
  }

  public double getHighLaserCanDistance() {
    // returns the distance from the laserCAN in millimeters
    Measurement measurement = highLaser.getMeasurement();
    highLaserOnline = measurement != null;
    return getDistance(measurement);
  }

  private double getDistance(Measurement measurement) {
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement.distance_mm;
    } else {
      return 1000;
    }
  }

  public boolean lowLaserCanOnline() {
    return lowLaserOnline;
  }

  public boolean highLaserCanOnline() {
    return highLaserOnline;
  }

  public double getOutputCurrent() {
    return highIndexMotor.getOutputCurrent();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("High LaserCAN Distance", getHighLaserCanDistance());
    SmartDashboard.putNumber("Low LaserCAN Distance", getLowLaserCanDistance());

    SmartDashboard.putBoolean("Have Note", isStored());
  }
}
