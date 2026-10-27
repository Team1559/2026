package frc.lib.component;

import org.wpilib.units.measure.AngularVelocity;

import frc.lib.logging.LoggableComponent;
import frc.lib.util.NeutralOutput;

public interface AngularVelocityComponent extends LoggableComponent, NeutralOutput {
    void setVelocity(AngularVelocity setpoint);

    AngularVelocity getCurrentVelocity();
}
