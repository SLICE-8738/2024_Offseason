// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

/** 
 * Directly sets the drive and angle motors of all drivetrain swerve modules
 * to given drive and angle motor percent outputs.
 */
public class SetPercentOutputCommand extends Command {

  private final Drivetrain m_drivetrain;

  private final double drivePercentOutput, anglePercentOutput;

  /** Creates a new SetPercentOutputCommand. 
   * 
   * @param drivetrain The instance of the Drivetrain subsystem declared in RobotContainer.
   * @param drivePercentOutput The desired percent output between -1 and 1 to set all drivetrain drive motors to.
   * @param anglePercentOutput The desired percent output between -1 and 1 to set all drivetrain angle motors to.
   */
  public SetPercentOutputCommand(Drivetrain drivetrain, double drivePercentOutput, double anglePercentOutput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    this.drivePercentOutput = drivePercentOutput;
    this.anglePercentOutput = anglePercentOutput;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivetrain.setPercentOutput(drivePercentOutput, anglePercentOutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.setPercentOutput(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }

}