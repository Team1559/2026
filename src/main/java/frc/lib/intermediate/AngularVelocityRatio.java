package frc.lib.intermediate;

import edu.wpi.first.units.measure.AngularVelocity;

import frc.lib.component.AngularVelocityComponent;
import frc.lib.logging.LoggableIntermediate;

public class AngularVelocityRatio extends LoggableIntermediate implements AngularVelocityComponent {
    private final double reductionRatio;
    private final AngularVelocityComponent child;

    /**The reduction ratio is in motor rotations over mechanism rotations. */
    public AngularVelocityRatio(double reductionRatio, AngularVelocityComponent child) {
        this.reductionRatio = reductionRatio;
        this.child = child;
        this.setChild(child);
    }

    @Override
    public void setVelocity(AngularVelocity setpoint) {
        child.setVelocity(setpoint.times(reductionRatio));
    }

    @Override
    public double getMotorCurrent() {
        return child.getMotorCurrent();
    }

    @Override
    public double getMotorTemperature() {
        return child.getMotorTemperature();
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
