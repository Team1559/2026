package frc.lib.angularPosition;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LoggableComponent;

public interface AngularPositionComponent extends LoggableComponent {
    
    void setAngle(Rotation2d angle);
    
    Rotation2d getAngle();
    
}
