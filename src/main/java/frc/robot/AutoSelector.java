// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
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

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * This class primarily manages the creation and updating of the autonomous mode
 * and starting position {@link SendableChooser sendable choosers} on Shuffleboard.
 */
public class AutoSelector {

    public enum StartingPosition {

        AMP_SIDE("Amp Side"),
        AMP_SIDE_CORNER("Amp Side Corner"),
        MIDDLE("Middle"),
        SOURCE_SIDE("Source Side"),
        FAR_SOURCE_SIDE("Far Source Side");

        public final String value;

        StartingPosition(String value) {

            this.value = value;

        }

    }

    public enum Mode {

        SCORE_7_SPEAKER("Score 7 Speaker", true),
        SCORE_5_SPEAKER_1("Score 5 Speaker (1, 2, 3, 5)", true),
        SCORE_5_SPEAKER_2("Score 5 Speaker (3, 2, 1, 5)", true),
        SCORE_5_SPEAKER_3("Score 5 Speaker (2, 3, 1, 5)", true),
        SCORE_4_SPEAKER("Score 4 Speaker", true),
        SCORE_3_AND_A_HALF_SPEAKER("Score 3.5 Speaker", true),
        SCORE_3_SPEAKER_1("Score 3 Speaker (1 & 2)", true),
        SCORE_3_SPEAKER_2("Score 3 Speaker (7 & 8)", true),
        SCORE_3_SPEAKER_3("Score 3 Speaker (8 & 7)", true),
        SCORE_3_SPEAKER_4("Score 3 Speaker (6 & 7)", true),
        SCORE_2_SPEAKER("Score 2 Speaker", true),
        SCORE_1_SPEAKER("Score 1 Speaker", false),
        CHOREO_TEST_AUTO("Choreo Test Auto", false),
        CHOREO_TEST_PATH("Choreo Test Path", false),
        TEST_PATH("Test Path", false);

        public final String value;
        public final boolean useStartingPosition;

        Mode(String value, boolean useStartingPosition) {

            this.value = value;
            this.useStartingPosition = useStartingPosition;

        }

    }

    private StartingPosition storedStartingPosition = StartingPosition.AMP_SIDE;
    private Mode storedMode = Mode.SCORE_1_SPEAKER;

    public SendableChooser<StartingPosition> startingPositionChooser;
    public SendableChooser<Mode> modeChooser;

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

        startingPositionChooser.setDefaultOption("Amp Side", StartingPosition.AMP_SIDE);

        startingPositionChooser.addOption("Middle", StartingPosition.MIDDLE);
        startingPositionChooser.addOption("Source Side", StartingPosition.SOURCE_SIDE);
        startingPositionChooser.addOption("Far Source Side", StartingPosition.FAR_SOURCE_SIDE);

        startingPositionChooser.onChange((position) -> updateAutoRoutine(position, modeChooser.getSelected()));

        modeChooser = new SendableChooser<Mode>();

        modeChooser.setDefaultOption(Mode.SCORE_1_SPEAKER.value, Mode.SCORE_1_SPEAKER);

        modeChooser.addOption(Mode.SCORE_7_SPEAKER.value, Mode.SCORE_7_SPEAKER);
        modeChooser.addOption(Mode.SCORE_5_SPEAKER_2.value, Mode.SCORE_5_SPEAKER_2);
        modeChooser.addOption(Mode.SCORE_5_SPEAKER_3.value, Mode.SCORE_5_SPEAKER_3);
        modeChooser.addOption(Mode.SCORE_4_SPEAKER.value, Mode.SCORE_4_SPEAKER);
        modeChooser.addOption(Mode.SCORE_3_AND_A_HALF_SPEAKER.value, Mode.SCORE_3_AND_A_HALF_SPEAKER);
        modeChooser.addOption(Mode.SCORE_3_SPEAKER_1.value, Mode.SCORE_3_SPEAKER_1);
        modeChooser.addOption(Mode.SCORE_3_SPEAKER_2.value, Mode.SCORE_3_SPEAKER_2);
        modeChooser.addOption(Mode.SCORE_3_SPEAKER_3.value, Mode.SCORE_3_SPEAKER_3);
        modeChooser.addOption(Mode.SCORE_3_SPEAKER_4.value, Mode.SCORE_3_SPEAKER_4);
        modeChooser.addOption(Mode.SCORE_2_SPEAKER.value, Mode.SCORE_2_SPEAKER);
        modeChooser.addOption(Mode.CHOREO_TEST_AUTO.value, Mode.CHOREO_TEST_AUTO);
        modeChooser.addOption(Mode.CHOREO_TEST_PATH.value, Mode.CHOREO_TEST_PATH);
        modeChooser.addOption(Mode.TEST_PATH.value, Mode.TEST_PATH);

        modeChooser.onChange((mode) -> updateAutoRoutine(startingPositionChooser.getSelected(), mode));

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
        NamedCommands.registerCommand("Note Align", new AlignWithNoteCommand(m_drivetrain, m_indexer).withTimeout(5));
        NamedCommands.registerCommand("Eject Note", new EjectNoteCommand(m_shooter, m_indexer));

    }

    private void updateAutoRoutine(StartingPosition position, Mode mode) {

        storedStartingPosition = position;
        storedMode = mode;

        try {

            System.out.println("Auto selection changed, updating creator; Starting Position: " + position.value
                + ", Mode: " + mode.value);
            autoRoutine = Optional.of(new PathPlannerAuto(mode.useStartingPosition? position.value + " " + mode.value : mode.value));

        }
        catch (Exception e) {

            DriverStation.reportError("Selected auto routine '" + (mode.useStartingPosition? position.value + " " + mode.value : mode.value)
                + "' does not exist", false);            
            autoRoutine = Optional.empty();

        }

    }

    public void updateInitialAutoPoseOffset() {

        Pose2d currentPose = m_drivetrain.getPose();

        try {

            initialAutoPose = PathPlannerAuto.getStaringPoseFromAutoFile(storedMode.useStartingPosition? 
                storedStartingPosition.value + " " + storedMode.value : storedMode.value);

        }
        catch (Exception e) {

            DriverStation.reportError("Selected auto routine '" + (storedMode.useStartingPosition? 
                storedStartingPosition.value + " " + storedMode.value : storedMode.value)+ "' does not exist", false);

        }

        if (currentPose != null && initialAutoPose != null) {

            Transform2d offset = initialAutoPose.minus(currentPose);

            initialAutoPoseXOffset = offset.getX();
            initialAutoPoseYOffset = offset.getY();
            initialAutoPoseRotationOffset = offset.getRotation().getDegrees();

        }

    }

    public PathPlannerAuto getAutoRoutine() {

        return autoRoutine.get();

    }

    public String getStartingPosition() {

        return startingPositionChooser.getSelected().value;

    }

    public String getMode() {

        return modeChooser.getSelected().value;

    }

}