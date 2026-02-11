package frc.lib.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LoggableIo;

public class VisionCameraIo extends LoggableIo<VisionCameraIo.VisionInputs> implements VisionComponent {
    @AutoLog
    public static abstract class VisionInputs implements LoggableInputs {
        public boolean isConnected = false;
        public TargetObservation latestTargetObservation;
        public int[] tagIds = new int[0];
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public Pose2d pose = Pose2d.kZero;
        public double timestamp;
        public boolean hasPose;
        public double stdevX;
        public double stdevY;
        public Rotation2d stdevRotation = Rotation2d.kZero;
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

    public TargetObservation getLatestTargetObservation() {
        return getInputs().latestTargetObservation;
    }

    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {

    }
    public static record PoseObservation(double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type) {

    }

    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }
}
