// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing.routines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PIDShooterTestCommand extends Command {
  /** Creates a new PIDShooterTestCommand. */
  private Shooter m_Shooter;
  private Timer timer;

  private int times;

  private int shooterPGain;
  private int shooterIGain;
  private int shooterDGain;

  private int aimPGain;
  private int aimIGain;
  private int aimDGain;

  private ShuffleboardTab testRoutine;
  private SimpleWidget currentFlywheelSpeedWidget; 


  public PIDShooterTestCommand(Shooter shooter, int sP, int sI, int sD, int aP, int aI, int aD) {
    m_Shooter = shooter;

    shooterPGain = sP;
    shooterIGain = sI;
    shooterDGain = sD;

    aimPGain = aP;
    aimIGain = aI;
    aimDGain = aD;

    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter);
    

    testRoutine = Shuffleboard.getTab("Test Routines");
    currentFlywheelSpeedWidget = testRoutine.add("Current Flywheel Speed", 0);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    m_Shooter.flyTopPID.setP(shooterPGain);
    m_Shooter.flyTopPID.setI(shooterIGain);
    m_Shooter.flyTopPID.setD(shooterDGain);
    m_Shooter.flyBottomPID.setP(shooterPGain);
    m_Shooter.flyBottomPID.setI(shooterIGain);
    m_Shooter.flyBottomPID.setD(shooterDGain);

    m_Shooter.aimPID.setP(aimPGain);
    m_Shooter.aimPID.setI(aimIGain);
    m_Shooter.aimPID.setD(aimDGain);

    m_Shooter.spinFlywheel(1);
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    times += 1;




    currentFlywheelSpeedWidget.getEntry().setDouble(m_Shooter.getFlywheelSpeed());

    if(timer.get() >= 2){
      isFinished();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.spinFlywheel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
