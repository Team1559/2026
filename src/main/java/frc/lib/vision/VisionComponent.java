package frc.lib.vision;

import java.lang.annotation.Target;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LoggableComponent;
import frc.lib.vision.VisionCameraIo.PoseObservation;
import frc.lib.vision.VisionCameraIo.TargetObservation;

public interface VisionComponent extends LoggableComponent {
    Pose2d getPose();

    double getTimestamp();

    boolean hasPose();

    double getStdevX();

    double getStdevY();

    Rotation2d getStdevRotation();

    int[] getTagIds();

    PoseObservation[] getPoseObservations();

    boolean isConnected();

    TargetObservation getLatestTargetObservation();
}
