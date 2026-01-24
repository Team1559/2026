package frc.lib.subsystems.velocity;

import frc.lib.subsystems.LoggableComponent;

public interface VelocityComponent extends LoggableComponent {
    void run(double TargetVelocity);

    void stop();

    double getMotorCurrent();

    double getMotorTemperature();

    double getCurrentVelocity();
}
