package frc.lib.angular_position;

import edu.wpi.first.units.measure.Angle;
import frc.lib.logging.LoggableIntermediate;

public class AngularPositionSensorOffset extends LoggableIntermediate implements AngularPositionSensor {
    private final Angle offset;
    private final AngularPositionSensor child;

    public AngularPositionSensorOffset(Angle offset, AngularPositionSensor child) {
        this.offset = offset;
        this.child = child;
        setChild(child);
    }

    @Override
    public Angle getAngle() {
        return child.getAngle().minus(offset);
    }
}
