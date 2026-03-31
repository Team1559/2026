package frc.lib.io;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.component.Gyro;
import frc.lib.logging.LoggableIo;
import frc.lib.swerve.GyroInputsAutoLogged;

public class Pigeon2IoBase extends LoggableIo<Pigeon2IoBase.GyroInputs> implements Gyro {
    @AutoLog
    public static abstract class GyroInputs implements LoggableInputs {
        public Rotation2d yaw = Rotation2d.kZero;
        public Rotation2d pitch = Rotation2d.kZero;
        public Rotation2d roll = Rotation2d.kZero;
    }

    public Pigeon2IoBase() {
        super(new GyroInputsAutoLogged());
    }

    @Override
    public Rotation2d getPitch() {
        return getInputs().pitch;
    }

    @Override
    public Rotation2d getRoll() {
        return getInputs().roll;
    }

    @Override
    public Rotation2d getYaw() {
        return getInputs().yaw;
    }
}
