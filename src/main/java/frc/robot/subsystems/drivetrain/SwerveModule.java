package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;
import frc.slicelibs.util.math.Conversions;
import frc.slicelibs.util.math.OnboardModuleState;

public class SwerveModule {

    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    public final int moduleNumber;
    private Rotation2d lastAngle;
    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(SwerveModuleIO io, int moduleNumber){
        this.io = io;
        this.moduleNumber = moduleNumber;        
        lastAngle = getState().angle;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain/Module" + Integer.toString(moduleNumber), inputs);
    }

    public void runSetpoint(SwerveModuleState state, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        state = OnboardModuleState.optimize(state, getState().angle);

        targetState = state;

        setAngle(state);
        setSpeed(state, isOpenLoop);
    }

    /** Runs the drive motor of the module with the specified voltage. */
    public void runCharacterization(double volts) {
        io.setDriveVoltage(volts);
    }

    /** Runs the module with the specified duty cycle percent outputs. */
    public void runDutyCycle(double drivePercentOutput, double anglePercentOutput) {
        io.setDriveDutyCycle(drivePercentOutput);
        io.setAngleDutyCycle(anglePercentOutput);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            io.setDriveDutyCycle(desiredState.speedMetersPerSecond / Constants.kDrivetrain.MAX_LINEAR_VELOCITY);
        }
        else {
            io.setDriveVelocity(desiredState.speedMetersPerSecond);
        }
    }
    
    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.kDrivetrain.MAX_LINEAR_VELOCITY * 0.01))
                ? lastAngle
                : desiredState.angle;

        io.setAnglePosition(angle.getDegrees());
        lastAngle = angle;
    }

    private Rotation2d getIntegratedAngle() {
        return inputs.integratedAnglePosition;
    }

    public Rotation2d getAbsoluteAngle() {
        return inputs.absoluteAnglePosition;
    }

    public void resetToAbsolute() {
        io.resetToAbsolute();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.talonToMPS(inputs.driveVelocityMetersPerSec, Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO), 
            getIntegratedAngle()
        );
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.talonToMeters(inputs.drivePositionMeters, Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO), 
            getIntegratedAngle()
        );
    }

    /**
     * @return The output current of the drive motor in amps
     */
    public double getDriveOutputCurrent() {
       return inputs.driveCurrentAmps;
    }
}