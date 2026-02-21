package frc.lib.velocity;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.loggable.LoggableIntermediate;

public class VelocityRatio extends LoggableIntermediate implements AngularVelocityComponent {
    private final double reductionRatio;
    private final AngularVelocityComponent child;

    /**The reduction ratio is in motor rotations over mechanism rotations. */
    public VelocityRatio(String name, double reductionRatio, AngularVelocityComponent child) {
        super(name);
        this.reductionRatio = reductionRatio;
        this.child = child;
        this.addChildren(child);
    }

    @Override
    public void setTargetVelocity(AngularVelocity targetVelocity) {
        child.setTargetVelocity(targetVelocity.times(reductionRatio));
    }

    @Override
    public void stop() {
        child.stop();
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
}
