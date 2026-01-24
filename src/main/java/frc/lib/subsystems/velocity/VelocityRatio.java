package frc.lib.subsystems.velocity;

import frc.lib.subsystems.LoggableIntermediate;

public class VelocityRatio extends LoggableIntermediate implements VelocityComponent{
    private final double reductionRatio;
    private final VelocityComponent child;

    public VelocityRatio(String name, double reductionRatio, VelocityComponent child) {
        super(name);
        this.reductionRatio = reductionRatio;
        this.child = child;
    }

    @Override
    public void run(double targetVelocity) {
        child.run(targetVelocity * reductionRatio);
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
    public double getCurrentVelocity() {
        return child.getCurrentVelocity() / reductionRatio;
    }
}
