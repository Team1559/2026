package frc.lib.subsystems.velocity;

public class VelocityRatioIo extends VelocityIo {
    private final double reductionRatio;
    private final VelocityIo child;
    public VelocityRatioIo(String name, double reductionRatio, VelocityIo child) {
        super(name);
        this.reductionRatio = reductionRatio;
        this.child = child;
    }

    @Override
    public void run(double targetVelocity) {
        super.run(targetVelocity);
        child.run(targetVelocity * reductionRatio);
    }

    @Override
    public void stop() {
        super.stop();
        child.stop();
    }

    @Override
    protected void updateInputs(VelocityInputs inputs) {
        child.getInputs();
        inputs.currentVelocity = child.getInputs().currentVelocity / reductionRatio;
        inputs.motorTemp = child.getInputs().motorTemp;
        inputs.motorCurrent = child.getInputs().motorCurrent;
    }
}
