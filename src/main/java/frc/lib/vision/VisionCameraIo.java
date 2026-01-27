package frc.lib.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LoggableIo;

public class VisionCameraIo extends LoggableIo<VisionCameraIo.VisionInputs> implements VisionComponent {
    @AutoLog
    public static abstract class VisionInputs implements LoggableInputs {
        public Pose2d pose;
        public double timestamp;
        public boolean hasPose;
        public double stdevX;
        public double stdevY;
        public Rotation2d stdevRotation;
    }

    public VisionCameraIo(String name) {
        super(name, new VisionInputsAutoLogged());
    }

    @Override
    public Pose2d getPose() {
        return getInputs().pose;
    }

    @Override
    public double getTimestamp() {
        return getInputs().timestamp;
    }

    @Override
    public boolean hasPose() {
        return getInputs().hasPose;
    }

    @Override
    public double getStdevX() {
        return getInputs().stdevX;
    }

    @Override
    public double getStdevY() {
        return getInputs().stdevY;
    }

    @Override
    public Rotation2d getStdevRotation() {
        return getInputs().stdevRotation;
    }
}
