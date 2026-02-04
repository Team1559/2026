package frc.lib.angularPosition;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.lib.LoggableComponent;

public interface AngularPositionComponent extends LoggableComponent {
    
    void setTargetAngle(Angle angle);
    
    double getAngle();
    
}
