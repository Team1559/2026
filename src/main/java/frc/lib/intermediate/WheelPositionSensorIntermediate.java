package frc.lib.intermediate;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Distance;

import frc.lib.component.AngleSensor;
import frc.lib.component.DistanceSensor;
import frc.lib.logging.LoggableIntermediate;

// I would not tolerate this class name if you ever had to actually type it
public class WheelPositionSensorIntermediate extends LoggableIntermediate implements DistanceSensor {

    private final AngleSensor sensor;
    private final Distance circumference;

    public WheelPositionSensorIntermediate(AngleSensor sensor, Distance wheelRadius) {
        this.sensor = sensor;
        this.setChild(sensor);
        circumference = wheelRadius.times(2 * Math.PI);

        
    }

    @Override
    public Distance getDistance() {
        return circumference.times(sensor.getAngle().in(Rotations));
    }
}
