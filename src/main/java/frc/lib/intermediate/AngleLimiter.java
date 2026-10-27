package frc.lib.intermediate;

import static org.wpilib.units.Units.Radians;

import org.wpilib.math.util.MathUtil;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;

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
                .of(Math.clamp(angle.in(Units.Radians), minAngle.in(Units.Radians), maxAngle.in(Units.Radians)));
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
