package frc.lib.angularPosition;

import edu.wpi.first.units.measure.Angle;
import frc.lib.LoggableComponent;

public interface AngularPositionSensor extends LoggableComponent{
    Angle getAngle();
}
