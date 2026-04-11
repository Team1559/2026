package frc.lib.intermediate;

import edu.wpi.first.units.measure.Angle;

import frc.lib.component.AngleSensor;
import frc.lib.logging.LoggableAdaptor;

public class AngleSensorOffsetter extends LoggableAdaptor<AngleSensor> implements AngleSensor {
    private final Angle offset;

    public AngleSensorOffsetter(Angle offset, AngleSensor child) {
        super(child);
        this.offset = offset;
    }

    @Override
    public Angle getAngle() {
        return child.getAngle().minus(offset);
    }
}
