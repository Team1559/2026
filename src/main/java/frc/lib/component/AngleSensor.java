package frc.lib.component;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import frc.lib.intermediate.AngleSensorOffsetter;
import frc.lib.intermediate.WheelPositionSensorIntermediate;
import frc.lib.logging.LoggableComponent;

public interface AngleSensor extends LoggableComponent{
    Angle getAngle();

    default AngleSensor withOffset(Angle offset) {
        return new AngleSensorOffsetter(offset, this);
    }

    default AngleSensor withOffset(Rotation2d offset) {
        return withOffset(offset.getMeasure());
    }

    default DistanceSensor withPositionWheelRadius(Distance radius){
        return new WheelPositionSensorIntermediate(this, radius);
    }
}
