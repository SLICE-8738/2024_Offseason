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

        AMP_SIDE("Amp Side"),
        MIDDLE("Middle"),
        SOURCE_SIDE("Source Side");

        public final String value;

        StartingPosition(String value) {

            this.value = value;

        }

    }

    public enum DesiredMode {

        TEST_PATH_MODE("Test Path");

        public final String value;

        DesiredMode(String value) {

            this.value = value;

        }

    }

    public static StartingPosition storedStartingPosition;
    public DesiredMode storedDesiredMode;

    public SendableChooser<StartingPosition> startingPositionChooser;
    public SendableChooser<DesiredMode> modeChooser;

    private Optional<PathPlannerAuto> autoRoutine = Optional.empty();
    private String autoName;

    private Pose2d initialAutoPose;

    public double initialAutoPoseXOffset = 0;
    public double initialAutoPoseYOffset = 0;
    public double initialAutoPoseRotationOffset = 0;

    private final Drivetrain m_drivetrain;

    public AutoSelector(Drivetrain drivetrain) {

        m_drivetrain = drivetrain;

        startingPositionChooser = new SendableChooser<StartingPosition>();

        startingPositionChooser.setDefaultOption("Amp Side", StartingPosition.AMP_SIDE);

        startingPositionChooser.addOption("Middle", StartingPosition.MIDDLE);
        startingPositionChooser.addOption("Source Side", StartingPosition.SOURCE_SIDE);

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

            autoRoutine = getAutoRoutineForParams(startingPosition, desiredMode);

            updateInitialAutoPoseOffset();

        }

        storedStartingPosition = startingPosition;
        storedDesiredMode = desiredMode;

    }

    private Optional<PathPlannerAuto> getAutoRoutineForParams(StartingPosition position, DesiredMode mode) {

        try {

            return Optional.of(new PathPlannerAuto(mode == DesiredMode.TEST_PATH_MODE? mode.value : position.value + " " + mode.value));

        }
        catch(Exception e) {

            DriverStation.reportError(e.getMessage(), true);
            return Optional.empty();

        }
 
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

        autoRoutine = Optional.empty();
        storedDesiredMode = null;

        initialAutoPose = null;

    }

    public PathPlannerAuto getAutoRoutine() {

        return autoRoutine.get();

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