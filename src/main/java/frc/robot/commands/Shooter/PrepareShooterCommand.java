// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Button;
import frc.robot.Constants;
import frc.robot.ShooterMath;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLimelight;

/**
 * Aims and spins up the flywheels to prepare for a shot into the speaker
 */
public class PrepareShooterCommand extends Command {
  private final Shooter m_shooter;
  private final Drivetrain m_drivetrain;
  private boolean flywheelsStopped;

  private boolean forceSubwoofer;

  private static final ShuffleboardTab shooterTestTab = Shuffleboard.getTab("Shooter Testing");
  // private final SimpleWidget distanceWidget, desiredAngleWidget, desiredSpeedWidget, currentFlywheelSpeed, topFlywheelCurrent, bottomFlywheelCurrent, multiplierWidget, originalVelocityWidget;
  private static final SimpleWidget originalVelocityWidget = shooterTestTab.add("Original Flywheel Velocity", 0);
  private static final SimpleWidget angleWidget = shooterTestTab.add("Angle Adjustment", 0);
  
  /** Creates a new ShootCommand. */
  public PrepareShooterCommand(Shooter shooter, Drivetrain drivetrain) {
    m_shooter = shooter;
    m_drivetrain  = drivetrain;

    // distanceWidget = shooterTestTab.add("Robot Distance", 0);
    // desiredAngleWidget = shooterTestTab.add("Desired Shooter Angle", 0);
    // desiredSpeedWidget = shooterTestTab.add("Desired Flywheel Speed", 0); 
    // topFlywheelCurrent = shooterTestTab.add("Top Flywheel Current", 0);
    // bottomFlywheelCurrent = shooterTestTab.add("Bottom Flywheel Current", 0);
    //multiplierWidget = shooterTestTab.add("Velocity Multiplier", 1);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    flywheelsStopped = true;

    forceSubwoofer = false;
  }

  public PrepareShooterCommand(Shooter shooter, Drivetrain drivetrain, boolean forceSubwoofer) {
    m_shooter = shooter;
    m_drivetrain  = drivetrain;

    // distanceWidget = shooterTestTab.add("Robot Distance", 0);
    // desiredAngleWidget = shooterTestTab.add("Desired Shooter Angle", 0);
    // desiredSpeedWidget = shooterTestTab.add("Desired Flywheel Speed", 0); 
    // topFlywheelCurrent = shooterTestTab.add("Top Flywheel Current", 0);
    // bottomFlywheelCurrent = shooterTestTab.add("Bottom Flywheel Current", 0);
    //multiplierWidget = shooterTestTab.add("Velocity Multiplier", 1);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    flywheelsStopped = true;

    this.forceSubwoofer = forceSubwoofer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_shooter.resetAimSpeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Determines distance to the speaker
    Translation2d distanceTranslation = DriverStation.isAutonomousEnabled()? ShooterLimelight.getTable().getSpeakerPosition() : m_drivetrain.getSpeakerPosition();
    double distanceToSpeaker = Math.hypot(distanceTranslation.getX(), distanceTranslation.getY());

    // distanceWidget.getEntry().setDouble(distanceToSpeaker);
    // Uses distance info the calculate optimal shot
    //ShotDetails shotDetails = ShooterMath.getShot(distanceToSpeaker);
    // Sets the flywheel speed and aim angle to the appropriate values 
    //double speed = shotDetails.getFlywheelVelocity();
    //originalVelocityWidget.getEntry().setDouble(speed);
    /*if (m_drivetrain.inShootingRange()) {
      m_shooter.spinFlywheels(speed);
      flywheelsStopped = false;
    }
    else if (!flywheelsStopped) {
      m_shooter.spinFlywheels(0);
      flywheelsStopped = true;
    }*/

    //speed *= multiplierWidget.getEntry().getDouble(1);
    double angle = -2.3;

    if(!forceSubwoofer && ShooterLimelight.getTable().getTargetDetected()){
      angle = ShooterMath.getDistanceBasedShooterAngle(distanceToSpeaker);
    }

    // double optionalAngleAdjust = Button.leftBumper2.getAsBoolean() ? 0 : -3.5;

    // double angleAdjust = distanceToSpeaker > 1.6 ? optionalAngleAdjust : 0;

    // angle += angleAdjust;

    double angleAdjust = angleWidget.getEntry().getDouble(0);
    //angle += angleAdjust;

    double speed = distanceToSpeaker < 1.303 ? 3500 : 4500;

    m_shooter.spinFlywheels(speed, false);
    m_shooter.aimShooter(angle);

    SmartDashboard.putNumber("Robot Distance", distanceToSpeaker);
    SmartDashboard.putNumber("Flywheel Target Velocity", speed);

    SmartDashboard.putNumber("Target Shooter Angle", angle);


    // desiredSpeedWidget.getEntry().setDouble(speed);
    // desiredAngleWidget.getEntry().setDouble(shotDetails.getShooterAngle());
    // topFlywheelCurrent.getEntry().setDouble(m_shooter.getTopOutputCurrent());
    // bottomFlywheelCurrent.getEntry().setDouble(m_shooter.getBottomOutputCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (!DriverStation.isAutonomousEnabled()) {
      m_shooter.stopFlywheels();
    }
    m_shooter.slowDownAim();
    m_shooter.aimShooter(Constants.kShooter.SHOOTER_STOW_ANGLE + 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}