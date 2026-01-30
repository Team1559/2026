package frc.lib.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.LoggableIo;

public class SwerveModuleIo extends LoggableIo<SwerveModuleIo.SwerveInputs> implements SwerveModule {
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

    @Override
    public void setSpeed(double speed) {
        Logger.recordOutput(getOutputLogPath("Speed"), speed);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        Logger.recordOutput(getOutputLogPath("Angle"), angle);
        Logger.recordOutput(getOutputLogPath("Timestamp"), Timer.getTimestamp());
    }

    @Override
    public Translation2d getLocation() {
        return location;
    }

    @Override
    public double getSpeed() {
        return getInputs().speed;
    }

    @Override
    public Rotation2d getAngle() {
        return getInputs().angle;
    }

    @Override
    public double getDistance() {
        return getInputs().distance;
    }

    @Override
    public double getSteerMotorTemp() {
        return getInputs().steerMotorTemp;
    }

    @Override
    public double getDriveMotorTemp() {
        return getInputs().driveMotorTemp;
    }

    @Override
    public double getSteerMotorCurrent() {
        return getInputs().steerMotorCurrent;
    }

    @Override
    public double getDriveMotorCurrent() {
        return getInputs().driveMotorCurrent;
    }
}