import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveTrainTestCommand extends Command{
    Drivetrain m_driveTrain;
    double[] m_currents;
    double m_averageCurrent;
    double m_maxCurrent;
    double[] m_speeds;
    double m_averageSpeed;

    public DriveTrainTestCommand(Drivetrain driveTrain){
        m_driveTrain = driveTrain;
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.setPercentOutput(0.0, 1.0);
    m_currents = m_driveTrain.driveOutputCurents();
    m_maxCurrent = m_currents[0];
    for (int i = 0; i < m_currents.length; i++){
      m_averageCurrent += m_currents[i];
      if(m_maxCurrent < m_currents[i]){
        m_maxCurrent = m_currents[i];
    }
    for (int i = 0; );
  }
    m_averageCurrent = m_averageCurrent / m_currents.length;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}