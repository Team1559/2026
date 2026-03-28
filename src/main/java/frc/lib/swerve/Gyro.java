package frc.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.logging.LoggableComponent;

public interface Gyro extends LoggableComponent {
    Rotation2d getYaw();
    Rotation2d getPitch();
    Rotation2d getRoll();
}
