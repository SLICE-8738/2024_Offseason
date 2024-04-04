// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeLimelight;

public class AlignWithNoteCommand extends Command {

  private final Drivetrain m_drivetrain;

  private final PIDController rotationController;

  /** Creates a new AlignWithNoteCommand. */
  public AlignWithNoteCommand(Drivetrain drivetrain) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

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
    boolean atSetpoint = rotationController.atSetpoint();
    double error = rotationController.getPositionError();
    double power = (40 - Math.abs(error)) / 20.0;

    m_drivetrain.swerveDrive(new Transform2d(-power, 0, Rotation2d.fromDegrees(-feedback)), false, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.swerveDrive(new Transform2d(), false, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
