// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightTable;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelights;
import frc.slicelibs.PolarJoystickFilter;
import frc.slicelibs.util.config.JoystickFilterConfig;

public class AprilTagAlignCommand extends Command {

  private final Drivetrain m_drivetrain;
  private final PS4Controller driverController;

  private final PolarJoystickFilter translationFilter;

  private final PIDController rotationAlignController, yAlignController;

  private final LimelightTable shooterLimelight;

  private boolean speakerDetected;

  /** Creates a new AprilTagAlignCommand. */
  public AprilTagAlignCommand(Drivetrain drivetrain, PS4Controller driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    this.driverController = driverController;

    translationFilter = new PolarJoystickFilter(new JoystickFilterConfig(
        0.07,
        0.85,
        Constants.OperatorConstants.driveExponent,
        Constants.OperatorConstants.driveExponentPercent));

    rotationAlignController = new PIDController(0, 0, 0);
    yAlignController = new PIDController(0, 0 ,0);

    rotationAlignController.setSetpoint(0);
    yAlignController.setSetpoint(0);

    shooterLimelight = Limelights.getShooterLimelight();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double aprilTagID = shooterLimelight.getAprilTagID();
    speakerDetected = (aprilTagID == 7 || aprilTagID == 8 || aprilTagID == 3 || aprilTagID == 4);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double[] translation = translationFilter.filter(driverController.getRawAxis(1), driverController.getRawAxis(0));

    double translationX = translation[0] * Constants.kDrivetrain.MAX_LINEAR_VELOCITY;
    double translationY = translation[1] * Constants.kDrivetrain.MAX_LINEAR_VELOCITY;

    m_drivetrain.swerveDrive(
      new Transform2d(
        translationX, 
        speakerDetected? translationY : yAlignController.calculate(-shooterLimelight.getYOffset()),
        Rotation2d.fromDegrees(rotationAlignController.calculate(-shooterLimelight.getXOffset()))),
      false, 
      true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.swerveDrive(new Transform2d(), false, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }
  
}
