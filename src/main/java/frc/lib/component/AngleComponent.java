package frc.lib.component;

import edu.wpi.first.units.measure.Angle;

import frc.lib.util.NeutralOutput;

public interface AngleComponent extends AngleSensor, NeutralOutput{
    void setAngle(Angle angle);

    void setPercievedAngle(Angle angle);
}
