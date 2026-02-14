package frc.lib.angularPosition;

import edu.wpi.first.units.measure.Angle;

public interface AngularPositionComponent extends AngularPositionSensor{
    void setTargetAngle(Angle angle);

    void setPercievedAngle(Angle angle);
}
