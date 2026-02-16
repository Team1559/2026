package frc.lib.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.loggable.LoggableIo;

public class GyroIo extends LoggableIo<GyroIo.GyroInputs> {
    @AutoLog
    public static abstract class GyroInputs implements LoggableInputs {
        public Rotation2d yaw = Rotation2d.kZero;
        public Rotation2d pitch = Rotation2d.kZero;
        public Rotation2d roll = Rotation2d.kZero;
    }

    public GyroIo(String name) {
        super(name, new GyroInputsAutoLogged());
    }
}
