package frc.lib.intermediate;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Distance;

import frc.lib.component.AngleSensor;
import frc.lib.component.DistanceSensor;
import frc.lib.logging.LoggableIntermediate;

// I would not tolerate this class name if you ever had to actually type it
public class WheelPositionSensorIntermediate extends LoggableIntermediate implements DistanceSensor {

    private Distance circumfrence;
    private AngleSensor sensor;

    public WheelPositionSensorIntermediate(AngleSensor sensor, Distance wheelRadius) {
        this.sensor = sensor;
        circumfrence = wheelRadius.times(2 * Math.PI);
    }


    @Override
    public Distance getDistance() {
        return Meters.of(sensor.getAngle().in(Rotations) * circumfrence.in(Meters));
    }

}
