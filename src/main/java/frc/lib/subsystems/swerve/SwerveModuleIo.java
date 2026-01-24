package frc.lib.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.subsystems.LoggableIo;

public class SwerveModuleIo extends LoggableIo<SwerveModuleIo.SwerveInputs> {
    @AutoLog
    public static abstract class SwerveInputs implements LoggableInputs {
        public double speed;
        public Rotation2d angle = Rotation2d.kZero;
        public double distance;
        public double steerMotorTemp;
        public double driveMotorTemp;
        public double steerMotorCurrent;
        public double driveMotorCurrent;
    }
    
    private final Translation2d location;

    public SwerveModuleIo(String name, Translation2d location) {
        super(name, new SwerveInputsAutoLogged());
        this.location = location;
    }

    /**
     * Sets the speed of the wheel.
     * 
     * @param speed the desired wheel speed in meters per second
     */
    public void setSpeed(double speed) {
        Logger.recordOutput(getOutputLogPath("Speed"), speed);
    }

    /**
     * Sets the target direction for the wheel 
     * @param angle the desired angle of the wheel 
     */
    public void setAngle(Rotation2d angle) {
        Logger.recordOutput(getOutputLogPath("Angle"), angle);
        Logger.recordOutput(getOutputLogPath("Timestamp"), Timer.getTimestamp());
    }
/**
 * @return The location of the wheel relative to the origin of the robot in meters
 */
    public Translation2d getLocation() {
        return location;
    }

    public void setState(SwerveModuleState state) {
        setAngle(state.angle);
        setSpeed(state.speedMetersPerSecond);
    }
}