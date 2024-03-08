package frc.robot.testing.routines;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveTrainTestCommand extends Command{
    Drivetrain driveTrain;
    double[] currents;
    double[] swivelAverageCurrent = {0,0,0,0};
    double[] driveAverageCurrent = {0,0,0,0};
    double[] swivelMaxCurrent = {0,0,0,0};
    double[] driveMaxCurrent = {0,0,0,0};
    SwerveModuleState[] speeds;
    double[] swivelAverageSpeed = {0,0,0,0};
    double[] driveAverageSpeed = {0,0,0,0};
    int swivelExecutes = 0;
    int driveExecutes = 0;

    private final Timer timer;

    public DriveTrainTestCommand(Drivetrain driveTrain){
        this.driveTrain = driveTrain;

        timer = new Timer();
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setPercentOutput(0.0, 1.0);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currents = driveTrain.driveOutputCurents();
    swivelExecutes += 1;
    for(int i = 0; i < 4; i++){
      swivelAverageCurrent[i] += currents[i];
    }
    for(int i = 0; i< 4; i++){
      if(swivelMaxCurrent[i] < currents[i]){
        swivelMaxCurrent[i] = currents[i];
      }
    }
    speeds = driveTrain.getStates();
    for(int i = 0; i < 4; i++){
      swivelAverageSpeed[i] += currents[i];
    }

    if(timer.get() >= 2){
      driveTrain.setPercentOutput(1.0, 0.0);
      while(timer.get() < 4){
        currents = driveTrain.driveOutputCurents();
        driveExecutes += 1;
        for(int i = 0; i < 4; i++){
          driveAverageCurrent[i] += currents[i];
        }
        for(int i = 0; i< 4; i++){
          if(driveMaxCurrent[i] < currents[i]){
            driveMaxCurrent[i] = currents[i];
          }
        }
        speeds = driveTrain.getStates();
        for(int i = 0; i < 4; i++){
            driveAverageSpeed[i] += currents[i];
        }
      }
      isFinished();
    }

}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setPercentOutput(0, 0);
    for(int i = 0; i < 4; i++){
      swivelAverageCurrent[i] /= swivelExecutes;
      swivelAverageSpeed[i] /= swivelExecutes;

      driveAverageCurrent[i] /= driveExecutes;
      driveAverageSpeed[i] /= driveExecutes;
    }
    System.out.println("Drivetrain Tests:");
    System.out.println("Average swivel output current: " + swivelAverageCurrent);
    System.out.println("Max swivel output current: " + swivelMaxCurrent);
    System.out.println("Average swivel speed: " + swivelAverageSpeed);

    System.out.println("Average drive output current: " + driveAverageCurrent);
    System.out.println("Max drive output current: " + driveMaxCurrent);
    System.out.println("Average drive Speed: " + driveAverageSpeed);
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}