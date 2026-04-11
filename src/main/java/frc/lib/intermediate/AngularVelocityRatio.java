package frc.lib.intermediate;

import edu.wpi.first.units.measure.AngularVelocity;

import frc.lib.component.AngularVelocityComponent;
import frc.lib.logging.LoggableAdaptor;

public class AngularVelocityRatio extends LoggableAdaptor<AngularVelocityComponent>
        implements AngularVelocityComponent {
    private final double reductionRatio;

    /** The reduction ratio is in motor rotations over mechanism rotations. */
    public AngularVelocityRatio(double reductionRatio, AngularVelocityComponent child) {
        super(child);
        this.reductionRatio = reductionRatio;
    }

    @Override
    public void setVelocity(AngularVelocity setpoint) {
        child.setVelocity(setpoint.times(reductionRatio));
    }

    @Override
    public AngularVelocity getCurrentVelocity() {
        return child.getCurrentVelocity().div(reductionRatio);
    }

    @Override
    public void neutralOutput() {
        child.neutralOutput();
    }
}
