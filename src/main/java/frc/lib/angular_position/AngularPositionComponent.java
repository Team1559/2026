package frc.lib.angular_position;

import edu.wpi.first.units.measure.Angle;
import frc.lib.NeutralOutput;

public interface AngularPositionComponent extends AngularPositionSensor, NeutralOutput{
    void setAngle(Angle angle);

    void setPercievedAngle(Angle angle);
}
