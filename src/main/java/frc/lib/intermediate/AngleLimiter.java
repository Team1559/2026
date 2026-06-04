package frc.lib.intermediate;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

import frc.lib.component.AngleComponent;
import frc.lib.logging.LoggableAdaptor;

public class AngleLimiter extends LoggableAdaptor<AngleComponent> implements AngleComponent {

    private final Angle minAngle;
    private final Angle maxAngle;

    public AngleLimiter(Angle minAngle, Angle maxAngle, AngleComponent child) {
        super(child);
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
    }

    @Override
    public void setAngle(Angle angle) {
        Angle setPoint = Radians
                .of(MathUtil.clamp(angle.in(Units.Radians), minAngle.in(Units.Radians), maxAngle.in(Units.Radians)));
        child.setAngle(setPoint);
    }

    @Override
    public Angle getAngle() {
        return child.getAngle();
    }

    @Override
    public void setPercievedAngle(Angle angle) {
        child.setPercievedAngle(angle);
    }

    @Override
    public void neutralOutput() {
        child.neutralOutput();
    }

}
