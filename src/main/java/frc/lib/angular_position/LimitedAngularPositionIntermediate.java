package frc.lib.angular_position;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.loggable.LoggableIntermediate;

public class LimitedAngularPositionIntermediate extends LoggableIntermediate implements AngularPositionComponent {

    private final Angle minAngle;
    private final Angle maxAngle;
    private final AngularPositionComponent child;

    public LimitedAngularPositionIntermediate(String name, Angle minAngle, Angle maxAngle, AngularPositionComponent child){
        super(name);
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.child = child;
        this.addChildren(child);
    }

    @Override
    public void setTargetAngle(Angle angle) {
        Angle setPoint = Radians.of(MathUtil.clamp(angle.in(Units.Radians), minAngle.in(Units.Radians), maxAngle.in(Units.Radians)));
        child.setTargetAngle(setPoint);
    }

    @Override
    public Angle getAngle() {
        return child.getAngle();
    }

    @Override
    public void setPercievedAngle(Angle angle) {
        child.setPercievedAngle(angle);
    }
    
}
