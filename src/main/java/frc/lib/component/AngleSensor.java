package frc.lib.component;

import edu.wpi.first.units.measure.Angle;

import frc.lib.logging.LoggableComponent;

public interface AngleSensor extends LoggableComponent{
    Angle getAngle();
}
