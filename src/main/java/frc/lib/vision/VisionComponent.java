package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.loggable.LoggableComponent;

public interface VisionComponent extends LoggableComponent {
    Pose2d getPose();

    double getTimestamp();

    boolean hasPose();

    double getStdevX();

    double getStdevY();

    Rotation2d getStdevRotation();
}
