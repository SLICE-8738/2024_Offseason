package frc.robot;

/**
 * A class for methods for calculating shooter trajectories
 */
public final class ShooterMath {
    
    public static class ShotDetails {
        private final double xVelocity, yVelocity;
        public ShotDetails(double xVelocity, double yVelocity) {
            this.xVelocity = xVelocity;
            this.yVelocity = yVelocity;
        }

        /**
         * @return the horizontal velocity of the note towards the speaker (in meters)
         */
        public double getXVelocity() {
          return xVelocity;
        }

        /**
         * @return the vertical velocity of the note (in meters)
         */
        public double getYVelocity() {
          return yVelocity;
        }
        
        /**
         * @return the total velocity of the note (in meters)
         */
        public double getLaunchVelocity() {
            return Math.hypot(xVelocity, yVelocity);
        }

        /**
         * @return the angle that the note is shot at relative to the ground (in degrees)
         */
        public double getLaunchAngle() {
            return Math.toDegrees(Math.atan(yVelocity/xVelocity));
        }

        /**
         * @return the angle of the line, from the pivot to the high flywheel, to the ground. (in degrees)
         */
        public double getShooterAngle() {
            return shooterAngleToLaunchAngle(getLaunchAngle());
        }

        public double getFlywheelVelocity() {
            return getLaunchVelocity();
        }
    }


  /**
   * @param shooterAngle the angle of the line, from the pivot to the high flywheel, to the ground. (in degrees)
   * @param robotX the distance from the robot (wherever it is measured from) to the speaker
   * @return the horizontal distance from the shooter to the speaker, in meters
   */
  public static double getShooterDistance(double shooterAngle, double robotDistance) {
    return Constants.kShooter.PIVOT_X + robotDistance + 0.3065 * Math.cos(Math.toRadians(shooterAngle - 8.622));
  }

  /**
   * @param shooterAngle the angle of the line, from the pivot to the high flywheel, to the ground. (in degrees)
   * @return the height of the shooter off the ground, in meters
   */
  public static double getShooterHeight(double shooterAngle) {
    return Constants.kShooter.PIVOT_Y + 0.3065 * Math.sin(Math.toRadians(shooterAngle - 8.622));
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
    return Constants.kShooter.LAUNCH_ANGLE_TO_SHOOTER_ANGLE - shooterAngle;
  }

  /**
   * Calculates the details of an optimal shot based on the height and distance of a shooter
   * @param distance the horizontal distance of the shooter from the speaker (in meters)
   * @param height the height of the shooter off the ground (in meters)
   * @return details of the optimal shot
   */
  public static ShotDetails getShot(double distance, double height) {
    // see https://www.desmos.com/calculator/sfjrd3ja6f
    double yVelocity = Math.sqrt(((80.5/12) - (3.28084 * height)) * 32 * 2); // y velocity of the note (ft/s) 
    double xVelocity = 32 * (3.28084 * distance) / yVelocity; // x velocity of the note (ft/s)
    // Convert back to meters
    yVelocity *= 0.3048;
    xVelocity *= 0.3048;

    return new ShotDetails(xVelocity, yVelocity);
  }

  /**
   * Calculates the details of an optimal shot based on only the distance of the robot from the speaker
   * @param robotDistance
   * @return
   */
  public static ShotDetails getShot(double robotDistance) {
    double distance = robotDistance + Constants.kShooter.PIVOT_X; // Initial guess for distance from the shooter to the speaker
    double height = Constants.kShooter.PIVOT_Y; // Initial guess for height of the shooter from the ground


    for (int i = 0; i < 3; i++) {
        // Calculate the shooter angle according to the current guess for shooter position
        double shooterAngle = getShot(distance, height).getShooterAngle();
        // Update the shooter distance and height based on the new angle
        distance = getShooterDistance(shooterAngle, robotDistance);
        height = getShooterHeight(shooterAngle);

        // Repeat as many times as nessecary to get an accurate shot
    }

    return getShot(distance, height);

  }
}
