package frc.lib.subsystems.velocity;

import frc.lib.subsystems.LoggableSubsystem;

public class VelocitySubsystem extends LoggableSubsystem {
    private final VelocityComponent[] children;
    private double targetVelocityRpm;
    private boolean running;

    public VelocitySubsystem(String name, VelocityComponent... children) {
        super(name);
        this.children = children;
        addChildren(children);
    }

    public void run(double velocityRpm) {
        running = true;
        targetVelocityRpm = velocityRpm;
    }

    public void stop() {
        running = false;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (running) {
            for (VelocityComponent i : children) {
                i.run(targetVelocityRpm);
            }
        } else {
            for (VelocityComponent i : children) {
                i.stop();
            }
        }
    }
}