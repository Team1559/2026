package frc.lib.component;

import edu.wpi.first.units.measure.Angle;

import frc.lib.intermediate.AngleSensorOffsetter;
import frc.lib.logging.LoggableComponent;

public interface AngleSensor extends LoggableComponent{
    Angle getAngle();

    default AngleSensor withOffset(Angle offset) {
        return new AngleSensorOffsetter(offset, this);
    }
}
