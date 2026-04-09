package frc.lib.intermediate;

import edu.wpi.first.units.measure.Angle;

import frc.lib.component.AngleSensor;
import frc.lib.logging.LoggableIntermediate;

public class AngleSensorOffsetter<T extends AngleSensor> extends LoggableIntermediate implements AngleSensor {
    protected final Angle offset;
    protected final T child;

    public AngleSensorOffsetter(Angle offset, T child) {
        this.offset = offset;
        this.child = child;
        setChild(child);
    }

    @Override
    public Angle getAngle() {
        return child.getAngle().minus(offset);
    }
}
