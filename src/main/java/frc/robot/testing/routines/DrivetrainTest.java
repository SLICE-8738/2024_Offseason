package frc.robot.testing.routines;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainTest extends Command{
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

    public DrivetrainTest(Drivetrain driveTrain){
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
    System.out.println(String.format("Average swivel output current: \nMotor A: %s \nMotor B: %s \nMotor C: %s \nMotor D; %s", swivelAverageCurrent[0], swivelAverageCurrent[1], swivelAverageCurrent[2], swivelAverageCurrent[3]));
    System.out.println(String.format("Max swivel output current: \nMotor A: %s \nMotor B: %s \nMotor C: %s \nMotor D; %s", swivelMaxCurrent[0], swivelMaxCurrent[1], swivelMaxCurrent[2], swivelMaxCurrent[3]));
    System.out.println(String.format("Average swivel speed:  \nMotor A: %s \nMotor B: %s \nMotor C: %s \nMotor D; %s", swivelAverageSpeed[0], swivelAverageSpeed[1], swivelAverageSpeed[2], swivelAverageSpeed[3]));

    System.out.println(String.format("Average drive output current: \nMotor A: %s \nMotor B: %s \nMotor C: %s \nMotor D; %s", driveAverageCurrent[0], driveAverageCurrent[1], driveAverageCurrent[2], driveAverageCurrent[3]));
    System.out.println(String.format("Max drive output current: \nMotor A: %s \nMotor B: %s \nMotor C: %s \nMotor D; %s", driveMaxCurrent[0], driveMaxCurrent[1], driveMaxCurrent[2], driveMaxCurrent[3]));
    System.out.println(String.format("Average drive Speed: \nMotor A: %s \nMotor B: %s \nMotor C: %s \nMotor D; %s", driveAverageSpeed[0], driveAverageSpeed[1], driveAverageSpeed[2], driveAverageSpeed[3]));
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}