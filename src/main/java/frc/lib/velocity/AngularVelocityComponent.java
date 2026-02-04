package frc.lib.velocity;

import frc.lib.LoggableComponent;

public interface AngularVelocityComponent extends LoggableComponent {
    void setTargetVelocity(double TargetVelocity);

    void stop();

    double getMotorCurrent();

    double getMotorTemperature();

    double getCurrentVelocity();
}
