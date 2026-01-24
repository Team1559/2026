package frc.lib.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LoggableIo;
import frc.lib.components.gyro.GyroInputsAutoLogged;

public class GyroIo extends LoggableIo<GyroIo.GyroInputs> {
    @AutoLog
    public static abstract class GyroInputs implements LoggableInputs {
        public Rotation2d yaw;
        public Rotation2d pitch;
        public Rotation2d roll;
    }

    public GyroIo(String name) {
        super(name, new GyroInputsAutoLogged());
    }
}
