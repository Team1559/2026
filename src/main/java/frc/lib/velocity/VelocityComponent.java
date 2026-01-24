package frc.lib.velocity;

import frc.lib.LoggableComponent;

public interface VelocityComponent extends LoggableComponent {
    void run(double TargetVelocity);

    void stop();

    double getMotorCurrent();

    double getMotorTemperature();

    double getCurrentVelocity();
}
