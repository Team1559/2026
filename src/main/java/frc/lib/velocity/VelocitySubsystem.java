package frc.lib.velocity;

import frc.lib.LoggableSubsystem;

public class VelocitySubsystem extends LoggableSubsystem {
    private final AngularVelocityComponent[] children;
    private double targetVelocityRpm;
    private boolean running;

    public VelocitySubsystem(String name, AngularVelocityComponent... children) {
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
            for (AngularVelocityComponent i : children) {
                i.setTargetVelocity(targetVelocityRpm);
            }
        } else {
            for (AngularVelocityComponent i : children) {
                i.stop();
            }
        }
    }
}