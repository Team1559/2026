package frc.lib.component;

import edu.wpi.first.units.measure.LinearVelocity;

import frc.lib.logging.LoggableComponent;
import frc.lib.util.NeutralOutput;

public interface LinearVelocityComponent extends LoggableComponent, NeutralOutput {
    void setVelocity(LinearVelocity setpoint);

    LinearVelocity getCurrentVelocity();
}
