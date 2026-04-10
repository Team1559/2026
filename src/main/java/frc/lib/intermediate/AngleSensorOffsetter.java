package frc.lib.intermediate;

import edu.wpi.first.units.measure.Angle;

import frc.lib.component.AngleSensor;
import frc.lib.logging.LoggableIntermediate;

public class AngleSensorOffsetter extends LoggableIntermediate implements AngleSensor {
    private final Angle offset;
    private final AngleSensor child;

    public AngleSensorOffsetter(Angle offset, AngleSensor child) {
        this.offset = offset;
        this.child = child;
        setChild(child);
    }

    @Override
    public Angle getAngle() {
        return child.getAngle().minus(offset);
    }
}
