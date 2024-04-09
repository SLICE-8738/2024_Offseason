// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.commands.Drivetrain.AlignWithNoteCommand;
import frc.robot.commands.Intake.StoreNote.StoreNoteSequence;
import frc.robot.commands.Shooter.EjectNoteCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinFlywheelsCommand;
import frc.robot.commands.Shooter.SubwooferShotCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLimelight;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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

        SCORE_5_SPEAKER_1("Score 5 Speaker (Notes 1, 2, 3, 5)", true),
        SCORE_5_SPEAKER_2("Score 5 Speaker (Notes 3, 2, 1, 5)", true),
        SCORE_5_SPEAKER_3("Score 5 Speaker (Notes 2, 3, 1, 5)", true),
        SCORE_4_SPEAKER("Score 4 Speaker", true),
        SCORE_1_AMP_AND_3_SPEAKER("Score 1 Amp And 3 Speaker", true),
        SCORE_3_SPEAKER_1("Score 3 Speaker (Notes 1 & 2)", true),
        SCORE_3_SPEAKER_2("Score 3 Speaker (Notes 7 & 8)", true),
        SCORE_3_SPEAKER_3("Score 3 Speaker (Notes 8 & 7)", true),
        SCORE_1_AMP_AND_2_SPEAKER("Score 1 Amp And 2 Speaker", true),
        SCORE_2_SPEAKER("Score 2 Speaker", true),
        SCORE_1_SPEAKER("Score 1 Speaker", false),
        TEST_PATH_MODE("Test Path", false);

        public final String value;
        public final boolean useStartingPosition;

        DesiredMode(String value, boolean useStartingPosition) {

            this.value = value;
            this.useStartingPosition = useStartingPosition;

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
    private final Shooter m_shooter;
    private final Intake m_intake;
    private final Indexer m_indexer;

    public AutoSelector(Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {

        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_intake = intake;
        m_indexer = indexer;

        startingPositionChooser = new SendableChooser<StartingPosition>();


        startingPositionChooser.setDefaultOption("Middle", StartingPosition.MIDDLE);

        startingPositionChooser.addOption("Source Side", StartingPosition.SOURCE_SIDE);
        startingPositionChooser.addOption("Amp Side", StartingPosition.AMP_SIDE);

        modeChooser = new SendableChooser<DesiredMode>();

        modeChooser.setDefaultOption(DesiredMode.SCORE_5_SPEAKER_1.value, DesiredMode.SCORE_5_SPEAKER_1);

        modeChooser.addOption(DesiredMode.SCORE_5_SPEAKER_2.value, DesiredMode.SCORE_5_SPEAKER_2);
        modeChooser.addOption(DesiredMode.SCORE_5_SPEAKER_3.value, DesiredMode.SCORE_5_SPEAKER_3);
        modeChooser.addOption(DesiredMode.SCORE_4_SPEAKER.value, DesiredMode.SCORE_4_SPEAKER);
        modeChooser.addOption(DesiredMode.SCORE_2_SPEAKER.value, DesiredMode.SCORE_2_SPEAKER);
        modeChooser.addOption(DesiredMode.SCORE_1_AMP_AND_3_SPEAKER.value, DesiredMode.SCORE_1_AMP_AND_3_SPEAKER);
        modeChooser.addOption(DesiredMode.SCORE_3_SPEAKER_1.value, DesiredMode.SCORE_3_SPEAKER_1);
        modeChooser.addOption(DesiredMode.SCORE_3_SPEAKER_1.value, DesiredMode.SCORE_3_SPEAKER_2);
        modeChooser.addOption(DesiredMode.SCORE_3_SPEAKER_1.value, DesiredMode.SCORE_3_SPEAKER_3);
        modeChooser.addOption(DesiredMode.SCORE_1_AMP_AND_2_SPEAKER.value, DesiredMode.SCORE_1_AMP_AND_2_SPEAKER);
        modeChooser.addOption(DesiredMode.SCORE_1_SPEAKER.value, DesiredMode.SCORE_1_SPEAKER);
        modeChooser.addOption(DesiredMode.TEST_PATH_MODE.value, DesiredMode.TEST_PATH_MODE);

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

        NamedCommands.registerCommand("Store Note", new StoreNoteSequence(m_indexer, m_intake));
        NamedCommands.registerCommand("Shoot Note", new ShootCommand(m_shooter, m_indexer, m_drivetrain));
        NamedCommands.registerCommand("Shoot Note Subwoofer", new SubwooferShotCommand(m_shooter, m_indexer, m_drivetrain));
        NamedCommands.registerCommand("Spin Flywheels", new SpinFlywheelsCommand(m_shooter, 3500));
        NamedCommands.registerCommand("Note Align", new AlignWithNoteCommand(m_drivetrain, m_indexer).onlyIf(() -> ShooterLimelight.getTable().getTargetDetected()).withTimeout(5));
        NamedCommands.registerCommand("Eject Note", new EjectNoteCommand(m_shooter, m_indexer));

    }

    public void updateAutoSelector() {

        StartingPosition startingPosition = startingPositionChooser.getSelected();
        DesiredMode desiredMode = modeChooser.getSelected();

        if (storedStartingPosition != startingPosition || storedDesiredMode != desiredMode) {

            System.out.println("Auto selection changed, updating creator; Starting Position: " + startingPosition.name()
                    + ", Desired Mode: " + desiredMode.name());

            autoRoutine = getAutoRoutineForParams(startingPosition, desiredMode);

            updateInitialAutoPoseOffset(startingPosition, desiredMode);

        }

        storedStartingPosition = startingPosition;
        storedDesiredMode = desiredMode;

    }

    private Optional<PathPlannerAuto> getAutoRoutineForParams(StartingPosition position, DesiredMode mode) {

        try {

            return Optional.of(new PathPlannerAuto(mode.useStartingPosition? mode.value : position.value + " " + mode.value));

        }
        catch (Exception e) {

            DriverStation.reportError(e.getMessage(), true);
            return Optional.empty();

        }

    }

    public void updateInitialAutoPoseOffset(StartingPosition position, DesiredMode mode) {

        Pose2d currentPose = m_drivetrain.getPose();

        initialAutoPose = PathPlannerAuto.getStaringPoseFromAutoFile(mode == DesiredMode.TEST_PATH_MODE? mode.value : position.value + " " + mode.value);

        if (currentPose != null && initialAutoPose != null) {

            Transform2d offset = initialAutoPose.minus(currentPose);

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