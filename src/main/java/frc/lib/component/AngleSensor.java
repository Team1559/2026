package frc.lib.component;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;

import frc.lib.intermediate.AngleSensorOffsetter;
import frc.lib.logging.LoggableComponent;

public interface AngleSensor extends LoggableComponent{
    Angle getAngle();

    default AngleSensor withOffset(Angle offset) {
        return new AngleSensorOffsetter(offset, this);
    }

    default AngleSensor withOffset(Rotation2d offset) {
        return withOffset(offset.getMeasure());
    }
}
