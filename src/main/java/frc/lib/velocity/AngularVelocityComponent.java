package frc.lib.velocity;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.NeutralOutput;
import frc.lib.logging.LoggableComponent;

public interface AngularVelocityComponent extends LoggableComponent, NeutralOutput {
    void setVelocity(AngularVelocity setpoint);
   
    double getMotorCurrent();

    double getMotorTemperature();

    AngularVelocity getCurrentVelocity();
}
