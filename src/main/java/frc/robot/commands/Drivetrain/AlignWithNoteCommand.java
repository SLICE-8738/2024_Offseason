// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeLimelight;

public class AlignWithNoteCommand extends Command {

  private final Drivetrain m_drivetrain;

  private final PIDController rotationController;

  private final Indexer m_indexer;

  /** Creates a new AlignWithNoteCommand. */
  public AlignWithNoteCommand(Drivetrain drivetrain, Indexer indexer) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    m_indexer = indexer;

    rotationController = new PIDController(Constants.kDrivetrain.kPNoteAlignRotation, Constants.kDrivetrain.kINoteAlignRotation, Constants.kDrivetrain.kDNoteAlignRotation);
    rotationController.setSetpoint(0);
    rotationController.setTolerance(2);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double feedback = rotationController.calculate(IntakeLimelight.getTable().getXOffset());
    double error = rotationController.getPositionError();
    double power = (40 - Math.abs(error)) / 20.0;

    double height = IntakeLimelight.getTable().getYOffset() - 18.8;
    double multiplier = (height + 20) / 15.0;
    multiplier = MathUtil.clamp(multiplier, 0.45, 2);

    multiplier = IntakeLimelight.getTable().getTargetDetected() ? multiplier : 0.3;

    power *= multiplier;

    m_drivetrain.swerveDrive(new Transform2d(-power, 0, Rotation2d.fromDegrees(-feedback)), false, false);
    LimelightHelpers.setLEDMode_ForceOn("limelight-intake");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.swerveDrive(new Transform2d(), false, false);
    LimelightHelpers.setLEDMode_ForceOff("limelight-intake");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_indexer.getLaserCanDistance() < Constants.kIndexer.DEFAULT_LASERCAN_DISTANCE;
  }

}
