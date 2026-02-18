package frc.lib.velocity;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.LoggableComponent;

public interface AngularVelocityComponent extends LoggableComponent {
    void setTargetVelocity(AngularVelocity TargetVelocity);

    void stop();

    double getMotorCurrent();

    double getMotorTemperature();

    AngularVelocity getCurrentVelocity();
}
