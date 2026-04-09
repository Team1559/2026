package frc.lib.intermediate;

import edu.wpi.first.units.measure.Angle;

import frc.lib.component.AngleComponent;

public class AngleComponentOffsetter<T extends AngleComponent> extends AngleSensorOffsetter<T>
        implements AngleComponent {

    public AngleComponentOffsetter(Angle offset, T child) {
        super(offset, child);
    }

    @Override
    public void neutralOutput() {
        child.neutralOutput();
    }

    @Override
    public void setAngle(Angle angle) {
        child.setAngle(angle.plus(offset));
    }

    @Override
    public void setPercievedAngle(Angle angle) {
        child.setPercievedAngle(angle.plus(offset));
    }

}
