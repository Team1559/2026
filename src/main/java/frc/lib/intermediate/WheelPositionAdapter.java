package frc.lib.intermediate;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Distance;

import frc.lib.component.AngleSensor;
import frc.lib.component.DistanceSensor;
import frc.lib.logging.LoggableAdapter;

// I would not tolerate this class name if you ever had to actually type it
public class WheelPositionAdapter extends LoggableAdapter<AngleSensor> implements DistanceSensor {

    private final Distance circumference;

    public WheelPositionAdapter(AngleSensor sensor, Distance wheelRadius) {
        super(sensor);
        circumference = wheelRadius.times(2 * Math.PI);

        
    }

    @Override
    public Distance getDistance() {
        return circumference.times(child.getAngle().in(Rotations));
    }
}
