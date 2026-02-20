package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.LoggableComponent;

public interface VisionComponent extends LoggableComponent {
    int[] getTagIds();

    PoseObservation[] getPoseObservations();

    boolean isConnected();

    public record PoseObservation(double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type) {
    }

    public enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2
    }

}
