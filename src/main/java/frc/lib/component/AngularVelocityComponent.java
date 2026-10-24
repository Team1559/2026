package frc.lib.component;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import frc.lib.intermediate.AngularVelocityRatio;
import frc.lib.logging.LoggableComponent;
import frc.lib.util.NeutralOutput;

public interface AngularVelocityComponent extends LoggableComponent, NeutralOutput {
    void setVelocity(AngularVelocity setpoint);

    AngularVelocity getCurrentVelocity();

    default AngularVelocityComponent withVelocityRatio(double reductionRatio){
        return new AngularVelocityRatio(reductionRatio, this);
    }
}
