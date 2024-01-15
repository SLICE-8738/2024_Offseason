// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.subsystems.Drivetrain;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * This class primarily manages the creation and updating of the autonomous mode
 * and starting position sendable choosers on Shuffleboard.
 * 
 * <p>
 * {@link SendableChooser See SendableChooser class here}
 */
public class AutoSelector {

    public enum StartingPosition {

        LEFT,
        MIDDLE,
        RIGHT

    }

    public enum DesiredMode {

        TEST_PATH_MODE

    }

    public static StartingPosition storedStartingPosition;
    public DesiredMode storedDesiredMode;

    public SendableChooser<StartingPosition> startingPositionChooser;
    public SendableChooser<DesiredMode> modeChooser;

    private Optional<PathPlannerAuto> autoMode = Optional.empty();
    private String autoName;

    private Pose2d initialAutoPose;

    public double initialAutoPoseXOffset = 0;
    public double initialAutoPoseYOffset = 0;
    public double initialAutoPoseRotationOffset = 0;

    private final Drivetrain m_drivetrain;

    public AutoSelector(Drivetrain drivetrain) {

        m_drivetrain = drivetrain;

        startingPositionChooser = new SendableChooser<StartingPosition>();

        startingPositionChooser.setDefaultOption("Left", StartingPosition.LEFT);

        startingPositionChooser.addOption("Middle", StartingPosition.MIDDLE);
        startingPositionChooser.addOption("Right", StartingPosition.RIGHT);

        modeChooser = new SendableChooser<DesiredMode>();

        //Add autonomous modes here with modeChooser.setDefaultOption() and modeChooser.addOption()
        modeChooser.setDefaultOption("Test Path", DesiredMode.TEST_PATH_MODE);

        AutoBuilder.configureHolonomic(
            m_drivetrain::getPose,
            m_drivetrain::resetOdometry,
            m_drivetrain::getChassisSpeeds,
            m_drivetrain::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.kAutonomous.kPTranslation),
                new PIDConstants(Constants.kAutonomous.kPRotation),
                Constants.kAutonomous.kMaxVelocityMetersPerSecond,
                Constants.kDrivetrain.DRIVE_BASE_RADIUS,
                new ReplanningConfig(false, false)),
            () -> DriverStation.getAlliance().get() == Alliance.Red,
            m_drivetrain);

        //Add custom commands to use in PathPlanner autos here with NamedCommands.registerCommand()

    }

    public void updateAutoSelector() {

        StartingPosition startingPosition = startingPositionChooser.getSelected();
        DesiredMode desiredMode = modeChooser.getSelected();

        if (storedStartingPosition != startingPosition || storedDesiredMode != desiredMode) {

            System.out.println("Auto selection changed, updating creator; Starting Position: " + startingPosition.name()
                    + ", Desired Mode: " + desiredMode.name());

            autoMode = getAutoModeForParams(startingPosition, desiredMode);

            updateInitialAutoPoseOffset();

        }

        storedStartingPosition = startingPosition;
        storedDesiredMode = desiredMode;

    }

    private Optional<PathPlannerAuto> getAutoModeForParams(StartingPosition position, DesiredMode mode) {

        switch (mode) {

            //Uncomment and replace this code with autonomous mode names
            /*case SCORE_ONE_HIGH_ROW:
                autoName = "Score One High Row";
                break;
            case SCORE_ONE_HIGH_ROW_AND_MOBILITY:
                autoName = "Score One High Row And Mobility";
                break;
            case SCORE_ONE_HIGH_ROW_AND_ENGAGE:
                autoName = "Score One High Row And Engage";
                break;
            case SCORE_ONE_HIGH_ROW_MOBILITY_AND_ENGAGE:
                autoName = "Score One High Row Mobility And Engage";
                break;
            case SCORE_ONE_HIGH_ROW_PICK_UP_AND_ENGAGE:
                autoName = "Score One High Row Pick Up And Engage";
                break;
            case SCORE_TWO_HIGH_AND_MID_ROW:
                autoName = "Score Two High And Mid Row";
                break;
            case SCORE_TWO_HIGH_AND_MID_ROW_AND_ENGAGE:
                autoName = "Score Two High And Mid Row And Engage";
                break;*/
            case TEST_PATH_MODE:
                autoName = "Test Auto";
                break;
            default:
                System.err.println("No valid auto mode found for " + mode);
                return Optional.empty();

        }

        return Optional.of(new PathPlannerAuto(autoName));

    }

    public void updateInitialAutoPoseOffset() {

        Pose2d botPose = m_drivetrain.getPose();

        initialAutoPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        if (botPose != null && initialAutoPose != null) {

            initialAutoPoseXOffset = Math.abs(initialAutoPose.getX() - botPose.getX());
            initialAutoPoseYOffset = Math.abs(initialAutoPose.getY() - botPose.getY());
            initialAutoPoseRotationOffset = initialAutoPose.getRotation().getDegrees() - botPose.getRotation().getDegrees();

        }

    }

    public void reset() {

        autoMode = Optional.empty();
        storedDesiredMode = null;

        initialAutoPose = null;

    }

    public PathPlannerAuto getAutoMode() {

        return autoMode.get();

    }

    public String getStoredDesiredMode() {

        if (storedDesiredMode != null) {

            return storedDesiredMode.name();

        } else {

            return "None Stored";

        }

    }

    public static StartingPosition getStoredStartingPosition() {

        return storedStartingPosition;

    }

    public String getStoredStartingPositionName() {

        if (storedStartingPosition != null) {

            return storedStartingPosition.name();

        } else {

            return "None Stored";

        }

    }

}