// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/** Contains and runs all code needed to display all necessary information on Shuffleboard.*/
public class ShuffleboardData {

    private final ShuffleboardTab driverTab, debugTab, modulesTab, autoTab;

    public ShuffleboardData(Drivetrain drivetrain, Shooter shooter, Intake intake, AutoSelector autoSelector) {

        driverTab = Shuffleboard.getTab("Driver Tab");
        debugTab = Shuffleboard.getTab("Debug Tab");
        modulesTab = Shuffleboard.getTab("Modules Tab");
        autoTab = Shuffleboard.getTab("Autonomous");

        new DrivetrainData(drivetrain, shooter);
        new ShooterData(shooter);
        new AutoData(autoSelector);
    }

    public class DrivetrainData {

        public DrivetrainData(Drivetrain drivetrain, Shooter shooter) {

            //Displays the current velocity in meters per second of the left front swerve module on Shuffleboard
            modulesTab.addDouble("Left Front Velocity", () -> drivetrain.getStates()[0].speedMetersPerSecond).
            withPosition(0, 0).
            withSize(2, 1);
            //Displays the current velocity in meters per second of the right front swerve module on Shuffleboard
            modulesTab.addDouble("Right Front Velocity", () -> drivetrain.getStates()[1].speedMetersPerSecond).
            withPosition(0, 3).
            withSize(2, 1);
            //Displays the current velocity in meters per second of the right back swerve module on Shuffleboard
            modulesTab.addDouble("Right Back Velocity", () -> drivetrain.getStates()[2].speedMetersPerSecond).
            withPosition(7, 0).
            withSize(2, 1);
            //Displays the current velocity in meters per second of the left back swerve module on Shuffleboard
            modulesTab.addDouble("Left Back Velocity", () -> drivetrain.getStates()[3].speedMetersPerSecond).
            withPosition(7, 3).
            withSize(2, 1);
        
            //Displays the current CANCoder angle in degrees with no offset of the left front swerve module on Shuffleboard
            modulesTab.addDouble("Left Front CANCoder Angle", () -> drivetrain.getCANCoderAngles()[0]).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", 0, "Max", 360)).
            withPosition(0, 1).
            withSize(2, 1);
            //Displays the current CANCoder angle in degrees with no offset of the left back swerve module on Shuffleboard
            modulesTab.addDouble("Left Back CANCoder Angle", () -> drivetrain.getCANCoderAngles()[1]).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", 0,"Max", 360)).
            withPosition(0, 2).
            withSize(2, 1);
            //Displays the current CANCoder angle in degrees with no offset of the right front swerve module on Shuffleboard
            modulesTab.addDouble("Right Front CANCoder Angle", () -> drivetrain.getCANCoderAngles()[2]).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", 0, "Max", 360)).
            withPosition(7, 1).
            withSize(2, 1);
            //Displays the current CANCoder angle in degrees with no offset of the right back swerve module on Shuffleboard
            modulesTab.addDouble("Right Back CANCoder Angle", () -> drivetrain.getCANCoderAngles()[3]).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", 0, "Max", 360)).
            withPosition(7, 2).
            withSize(2, 1);
        
            //Displays the current integrated encoder angle in degrees of the left front swerve module on Shuffleboard
            modulesTab.addDouble("Left Front Integrated Angle", () -> drivetrain.getStates()[0].angle.getDegrees()).
            withPosition(2, 0).
            withSize(2, 1);
            //Displays the current integrated encoder angle in degrees of the right front swerve module on Shuffleboard
            modulesTab.addDouble("Right Front Integrated Angle", () -> drivetrain.getStates()[1].angle.getDegrees()).
            withPosition(2, 3).
            withSize(2, 1);
            //Displays the current integrated encoder angle in degrees of the right back swerve module on Shuffleboard
            modulesTab.addDouble("Right Back Integrated Angle", () -> drivetrain.getStates()[2].angle.getDegrees()).
            withPosition(5, 0).
            withSize(2, 1);
            //Displays the current integrated encoder angle in degrees of the left back swerve module on Shuffleboard
            modulesTab.addDouble("Left Back Integrated Angle", () -> drivetrain.getStates()[3].angle.getDegrees()).
            withPosition(5, 3).
            withSize(2, 1);
        
            //Displays the current heading of the robot in degrees on Shuffleboard
            debugTab.addDouble("Drivetrain Heading", () -> drivetrain.getHeading().getDegrees()).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", 0, "Max", 360)).
            withPosition(0, 0).
            withSize(2, 1);
                
            //Displays the current position of the robot on the field on Shuffleboard
            debugTab.add(drivetrain.m_field2d).
            withPosition(4, 2).
            withSize(3, 2);
        
            //Displays the feed from the USB camera on Shuffleboard
            /*driverTab.add(CameraServer.startAutomaticCapture(0)).
            withWidget(BuiltInWidgets.kCameraStream).
            withPosition(1, 0).
            withSize(3, 3);*/

            //Displays the feed from the Limelight on Shuffleboard
            driverTab.addCamera("Limelight", "limelight-shooter-1", "http://10.87.38.67:5800").
            withPosition(0, 2).
            withSize(3, 3);

            driverTab.addCamera("Intake", "limelight-intake-1", "http://10.87.38.88:5800").
            withPosition(4, 0).
            withSize(7, 5);

            driverTab.addBoolean("Flywheel At Speed", () -> shooter.atTargetSpeed(Constants.kShooter.FLYWHEEL_RPM_ACCEPTABLE_ERROR)).
            withPosition(0, 0).
            withSize(4, 2);

            // //Adds a tuner for the drive motor PID gains to Shuffleboard
            // ShuffleboardTuner.create(
            //     (values) -> {
          
            //       drivetrain.setDrivePID(values[0], values[1], values[2]);
          
            //     },
            //     new String[] {"kP", "kI", "kD"},
            //     "Drive Motor PID");

            // //Adds a tuner for the angle motor PID gains to Shuffleboard
            // ShuffleboardTuner.create(
            //     (values) -> {

            //         drivetrain.setAnglePIDF(values[0], values[1], values[2], values[3]);

            //     },
            //     new String[] {"kP", "kI", "kD", "kFF"},
            //     "Angle Motor PIDF");

            // //Adds a tuner for the maximum velocities to Shuffleboard
            // ShuffleboardTuner.create(
            //     (values) -> {

            //         drivetrain.maxLinearVelocity = values[0];
            //         drivetrain.maxAngularVelocity = values[1];

            //     },
            //     new String[] {"Max Linear", "Max Angular"},
            //     "Drivetrain Max Velocities");

        }

    }

    public class ShooterData {

        public ShooterData(Shooter shooter) {

            //Displays the current absolute angle of the shooter pivot
            debugTab.addDouble("Shooter Absolute Angle", shooter::getAlternateAngle).
            withPosition(0, 3).
            withSize(2, 1);
            
        }

    }

    public class AutoData {

        public AutoData(AutoSelector autoSelector) {
        
            //Adds the sendable chooser for the desired autonomous mode onto Shuffleboard
            autoTab.add("Auto Mode", autoSelector.modeChooser).withPosition(2, 0).withSize(2, 1);
            //Adds the sendable chooser for the robot starting position onto Shuffleboard
            autoTab.add("Starting Position", autoSelector.startingPositionChooser).withPosition(5, 0).withSize(2, 1);

            //Displays the autonomous mode selected on the sendable chooser on Shuffleboard
            autoTab.addString("Selected Auto Mode", autoSelector::getMode).
            withPosition(2, 1).
            withSize(2, 1);
            //Displays the robot starting position selected on the sendable chooser on Shuffleboard
            autoTab.addString("Selected Starting Position", autoSelector::getStartingPosition).
            withPosition(5, 1).
            withSize(2, 1);

            //Displays the X offset of the robot from the inital pose of the selected autonomous routine on Shuffleboard
            autoTab.addDouble("Initial Auto Pose X Offset", () -> autoSelector.initialAutoPoseXOffset).
            withPosition(1, 2).
            withSize(2, 1);
            //Displays the Y offset of the robot from the inital pose of the selected autonomous routine on Shuffleboard
            autoTab.addDouble("Initial Auto Pose Y Offset", () -> autoSelector.initialAutoPoseYOffset).
            withPosition(6, 2).
            withSize(2, 1);
            //Displays the rotational offset of the robot from the inital pose of the selected autonomous routine on Shuffleboard
            autoTab.addDouble("Initial Auto Pose Rotation Offset", () -> autoSelector.initialAutoPoseRotationOffset).
            withPosition(3, 2).
            withSize(3, 1);

        }

    }

}