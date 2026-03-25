package frc.lib.angular_position;

import edu.wpi.first.units.measure.Angle;
import frc.lib.logging.LoggableIntermediate;

public class AngularPositionRatio extends LoggableIntermediate implements AngularPositionComponent {
    private final double reductionRatio;
    private final AngularPositionComponent child;

    public AngularPositionRatio(String name, double reductionRatio, AngularPositionComponent child) {
        super(name);
        this.reductionRatio = reductionRatio;
        this.child = child;
        this.addChildren(child);
    }

    @Override
    public Angle getAngle() {
        return child.getAngle().div(reductionRatio);
    }

    @Override
    public void setAngle(Angle angle) {
        child.setAngle(angle.times(reductionRatio));
    }

    @Override
    public void setPercievedAngle(Angle angle) {
        child.setPercievedAngle(angle.times(reductionRatio));
    }

    @Override
    public void neutralOutput() {
        child.neutralOutput();
    }
}
