package frc.lib.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.LoggableIo;

public class VisionCameraIo extends LoggableIo<VisionCameraIo.VisionInputs> implements VisionComponent {
    @AutoLog
    public static abstract class VisionInputs implements LoggableInputs {
        public boolean isConnected = false;
        public int[] tagIds = new int[0];
        public PoseObservation[] poseObservations = new PoseObservation[0];
    }

    public VisionCameraIo(String name) {
        super(name, new VisionInputsAutoLogged());
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
