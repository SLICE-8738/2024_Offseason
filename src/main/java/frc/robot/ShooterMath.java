package frc.robot;

/**
 * A class for methods for calculating shooter trajectories
 */
public final class ShooterMath {
    
    public class ShotDetails {
        private final double velocity, angle;
        public ShotDetails(double velocity, double angle) {
            this.velocity = velocity;
            this.angle = angle;
        }

        public double getLaunchVelocity() {
            return velocity;
        }

        public double getLaunchAngle() {
            return angle;
        }

        public double getShooterAngle() {
            return shooterAngleToLaunchAngle(angle);
        }

        public double getFlywheelVelocity() {
            return velocity;
        }
    }


  /**
   * @param shooterAngle the angle of the line, from the pivot to the high flywheel, to the ground. (in degrees)
   * @param robotX the distance from the robot (wherever it is measured from) to the speaker
   * @return the horizontal distance from the shooter to the speaker, in meters
   */
  public static double getShooterDistance(double shooterAngle, double robotDistance) {
    double highX = Constants.kShooter.DISTANCE_TO_HIGHER_FLYWHEEL * Math.cos(Math.toRadians(shooterAngle));
    double lowX = Constants.kShooter.DISTANCE_TO_LOWER_FLYWHEEL * Math.cos(Math.toRadians(shooterAngle - Constants.kShooter.ANGLE_BETWEEN_FLYWHEELS));
    return (-(highX + lowX) / 2) + Constants.kShooter.PIVOT_X + robotDistance;
  }

  /**
   * @param shooterAngle the angle of the line, from the pivot to the high flywheel, to the ground. (in degrees)
   * @return the height of the shooter off the ground, in meters
   */
  public static double getShooterHeight(double shooterAngle) {
    double highY = Constants.kShooter.DISTANCE_TO_HIGHER_FLYWHEEL * Math.sin(Math.toRadians(shooterAngle));
    double lowY = Constants.kShooter.DISTANCE_TO_LOWER_FLYWHEEL * Math.sin(Math.toRadians(shooterAngle - Constants.kShooter.ANGLE_BETWEEN_FLYWHEELS));
    return ((highY + lowY) / 2) + Constants.kShooter.PIVOT_Y; 
  }

  /**
   * Takes the angle that a note is launched and returns the appropriate angle of the shooter from the pivot
   * @param launchAngle the angle the note should be lanuched at
   * @return the angle of the line, from the pivot to the high flywheel, to the ground. (in degrees)
   */
  public static double launchAngleToShooterAngle(double launchAngle) {
    return Constants.kShooter.LAUNCH_ANGLE_TO_SHOOTER_ANGLE - launchAngle;
  }

  /**
   * Takes the angle that the shooter is at from the pivot and returns the angle the note will be launched at
   * @param shooterAngle the angle of the line, from the pivot to the high flywheel, to the ground. (in degrees)
   * @return the angle the note will be launched at given that shooter angle (in degrees)
   */
  public static double shooterAngleToLaunchAngle(double shooterAngle) {
    return Constants.kShooter.LAUNCH_ANGLE_TO_SHOOTER_ANGLE + shooterAngle;
  }

  public static double getLaunchAngle(double distance, double height) {
    // see https://www.desmos.com/calculator/sfjrd3ja6f
    double yVelocity = Math.sqrt(((80.5/12) - height) * 32 * 2); // y velocity of the note (ft/s) 
    double xVelocity = 32 * distance / yVelocity; // x velocity of the note (ft/s)
    double launchAngle = Math.atan(yVelocity / xVelocity); // Angle to launch the note at, in radians
    return launchAngle;
  }

  public static ShotDetails getShot(double robotDistance) {
    double distance = robotDistance + Constants.kShooter.PIVOT_X; // Initial guess for distance from the shooter to the speaker
    double height = Constants.kShooter.PIVOT_Y; // Initial guess for height of the shooter from the ground


    for (int i = 0; i < 2; i++) {
        // Get initial guess for launch angle of the note
        double launchAngle = getLaunchAngle(distance, height);
        // Find the corresponding angle for the shooter
        double shooterAngle = launchAngleToShooterAngle(launchAngle);
        // Update the shooter distance and height based on the new angle
        distance = getShooterDistance(shooterAngle, robotDistance);
        height = getShooterHeight(shooterAngle);

        // Repeat as many times as nessecary to get an accurate shot
    }

  }
}
