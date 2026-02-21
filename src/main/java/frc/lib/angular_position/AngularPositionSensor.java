package frc.lib.angular_position;

import edu.wpi.first.units.measure.Angle;
import frc.lib.loggable.LoggableComponent;

public interface AngularPositionSensor extends LoggableComponent{
    Angle getAngle();
}
