package frc.lib.component;

import edu.wpi.first.units.measure.AngularVelocity;

import frc.lib.logging.LoggableComponent;
import frc.lib.util.NeutralOutput;

public interface AngularVelocityComponent extends LoggableComponent, NeutralOutput {
    void setVelocity(AngularVelocity setpoint);
   
    double getMotorCurrent();

    double getMotorTemperature();

    AngularVelocity getCurrentVelocity();
}
