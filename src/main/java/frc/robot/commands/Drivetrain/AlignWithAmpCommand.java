// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.slicelibs.PolarJoystickFilter;
import frc.slicelibs.util.config.JoystickFilterConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Maintains driver control of robot translation, but automatically turns to face the speaker.
 */
public class AlignWithAmpCommand extends Command {

  private final Drivetrain m_drivetrain;

  private GenericHID m_driverController;
  private PolarJoystickFilter translationFilter;

  private final boolean m_isOpenLoop;
  private final boolean m_isFieldRelative;

  private final PIDController rotationController;

  /** Creates a new AlignWithSpeakerCommand for teleop. */
  public AlignWithAmpCommand(Drivetrain drivetrain, GenericHID driverController, boolean isOpenLoop,
      boolean isFieldRelative) {

    this(drivetrain, isOpenLoop, isFieldRelative);

    m_driverController = driverController;

    translationFilter = new PolarJoystickFilter(new JoystickFilterConfig(
        0.07,
        0.85,
        Constants.OperatorConstants.driveExponent,
        Constants.OperatorConstants.driveExponentPercent));

  }

  /** Creates a new AlignWithSpeakerCommand for autonomous. */
  public AlignWithAmpCommand(Drivetrain drivetrain, boolean isOpenLoop, boolean isFieldRelative) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    m_isOpenLoop = isOpenLoop;
    m_isFieldRelative = isFieldRelative;

    rotationController = new PIDController(Constants.kDrivetrain.kPSpeakerAlignRotation, Constants.kDrivetrain.kISpeakerAlignRotation, Constants.kDrivetrain.kDSpeakerAlignRotation);
    rotationController.enableContinuousInput(0, 360);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.setPercentOutput(0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double translationX = 0;
    double translationY = 0;

    if (m_driverController != null) {
    
      double[] translation = translationFilter.filter(m_driverController.getRawAxis(1), m_driverController.getRawAxis(0));

      translationX = translation[0] * Constants.kDrivetrain.MAX_LINEAR_VELOCITY;
      translationY = translation[1] * Constants.kDrivetrain.MAX_LINEAR_VELOCITY;

    }

    // Run PID Controller
    double currentAngle = m_drivetrain.getPose().getRotation().getDegrees();
    double turnAmount = rotationController.calculate(currentAngle, 90);

    m_drivetrain.swerveDrive(
        new Transform2d(new Translation2d(translationX, translationY), Rotation2d.fromDegrees(-turnAmount)),
        m_isOpenLoop,
        m_isFieldRelative);

    SmartDashboard.putNumber("Target Angle", 90);
    SmartDashboard.putNumber("Current Angle", currentAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.swerveDrive(
      new Transform2d(), 
      m_isOpenLoop,
      m_isFieldRelative);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(m_driverController.getRawAxis(2)) > 0.1;

  }

}