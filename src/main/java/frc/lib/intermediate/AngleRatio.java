package frc.lib.intermediate;

import edu.wpi.first.units.measure.Angle;

import frc.lib.component.AngleComponent;
import frc.lib.logging.LoggableAdaptor;

public class AngleRatio extends LoggableAdaptor<AngleComponent> implements AngleComponent {
    private final double reductionRatio;

    public AngleRatio(double reductionRatio, AngleComponent child) {
        super(child);
        this.reductionRatio = reductionRatio;
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
