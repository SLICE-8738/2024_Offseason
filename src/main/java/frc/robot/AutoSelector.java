// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
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

        TEST_PATH_MODE("Test Path"),
        SCORE_1_AMP_AND_INTAKE_NOTE("Score 1 Amp And Intake Note"),
        SCORE_1_SPEAKER_AND_INTAKE_NOTE("Score 1 Speaker And Intake Note");

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

        modeChooser.setDefaultOption(DesiredMode.TEST_PATH_MODE.value, DesiredMode.TEST_PATH_MODE);

        modeChooser.addOption(DesiredMode.SCORE_1_AMP_AND_INTAKE_NOTE.value, DesiredMode.SCORE_1_AMP_AND_INTAKE_NOTE);
        modeChooser.addOption(DesiredMode.SCORE_1_SPEAKER_AND_INTAKE_NOTE.value, DesiredMode.SCORE_1_SPEAKER_AND_INTAKE_NOTE);

        AutoBuilder.configureHolonomic(
            m_drivetrain::getPose,
            m_drivetrain::resetOdometry,
            m_drivetrain::getChassisSpeeds,
            m_drivetrain::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.kDrivetrain.TRANSLATION_KP),
                new PIDConstants(Constants.kDrivetrain.ROTATION_KP),
                Constants.kDrivetrain.MAX_MODULE_VELOCITY,
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

            updateInitialAutoPoseOffset(desiredMode);

        }

        storedStartingPosition = startingPosition;
        storedDesiredMode = desiredMode;

    }

    private Optional<PathPlannerAuto> getAutoRoutineForParams(StartingPosition position, DesiredMode mode) {

        try {

            return Optional.of(new PathPlannerAuto(mode == DesiredMode.TEST_PATH_MODE? mode.value : position.value + " " + mode.value));

        }
        catch(Exception e) {

            DriverStation.reportError(
                "Could not construct a valid PathPlannerauto for selected starting position and mode. Error: " + e.toString() + " " + e.getMessage(), true);
            return Optional.empty();

        }
 
    }

    public void updateInitialAutoPoseOffset(DesiredMode mode) {

        Pose2d botPose = m_drivetrain.getPose();

        initialAutoPose = PathPlannerAuto.getStaringPoseFromAutoFile(mode.value);

        if (botPose != null && initialAutoPose != null) {

            Transform2d offset = initialAutoPose.minus(botPose);

            initialAutoPoseXOffset = offset.getX();
            initialAutoPoseYOffset = offset.getY();
            initialAutoPoseRotationOffset = offset.getRotation().getDegrees();

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

            return storedDesiredMode.value;

        } else {

            return "None Stored";

        }

    }

    public String getStoredStartingPosition() {

        if (storedStartingPosition != null) {

            return storedStartingPosition.value;

        } else {

            return "None Stored";

        }

    }

}