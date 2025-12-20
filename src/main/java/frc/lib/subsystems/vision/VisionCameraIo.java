package frc.lib.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.lib.subsystems.LoggableIo;


public class VisionCameraIo extends LoggableIo<VisionCameraIo.VisionInputs> {
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
}
