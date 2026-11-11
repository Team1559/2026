package frc.lib.intermediate;

import edu.wpi.first.units.measure.Angle;

import frc.lib.component.AngleSensor;
import frc.lib.logging.LoggableAdapter;

public class AngleSensorOffsetter<T extends AngleSensor> extends LoggableAdapter<T> implements AngleSensor {
    protected final Angle offset;

    public AngleSensorOffsetter(Angle offset, T child) {
        super(child);
        this.offset = offset;
    }

    @Override
    public Angle getAngle() {
        return child.getAngle().minus(offset);
    }
}
