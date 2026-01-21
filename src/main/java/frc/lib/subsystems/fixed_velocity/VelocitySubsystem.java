package frc.lib.subsystems.fixed_velocity;

import frc.lib.subsystems.LoggableSubsystem;

public class VelocitySubsystem extends LoggableSubsystem {
    private final VelocityIo[] ios;
    private double targetVelocityRpm;
    private boolean running;

    public VelocitySubsystem(String name, VelocityIo... ios) {
        super(name);
        this.ios = ios;

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
            for (VelocityIo i : ios) {
                i.run(targetVelocityRpm);
            }
        } else {
            for (VelocityIo i : ios) {
                i.stop();
            }
        }
    }
}