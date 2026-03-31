package frc.lib.io;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.component.AprilTagSensor;
import frc.lib.logging.LoggableIo;
import frc.lib.vision.VisionInputsAutoLogged;

public class LimelightCameraIoBase extends LoggableIo<LimelightCameraIoBase.VisionInputs> implements AprilTagSensor {
    @AutoLog
    public static abstract class VisionInputs implements LoggableInputs {
        public boolean isConnected = false;
        public int[] tagIds = new int[0];
        public PoseObservation[] poseObservations = new PoseObservation[0];
    }

    public LimelightCameraIoBase() {
        super(new VisionInputsAutoLogged());
    }

    @Override
    public int[] getTagIds() {
        return getInputs().tagIds;
    }

    @Override
    public PoseObservation[] getPoseObservations() {
        return getInputs().poseObservations;
    }

    public boolean isConnected() {
        return getInputs().isConnected;
    }
}
