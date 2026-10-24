package frc.lib.intermediate;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import frc.lib.component.AngularVelocityComponent;
import frc.lib.component.LinearVelocityComponent;
import frc.lib.logging.LoggableAdapter;

public class WheelVelocityAdapter extends LoggableAdapter<AngularVelocityComponent> implements LinearVelocityComponent {

    private final Distance circumference;

    public WheelVelocityAdapter(AngularVelocityComponent wheel, Distance wheelRadius) {
        super(wheel);
        circumference = wheelRadius.times(2 * Math.PI);
    }

    @Override
    public void neutralOutput() {
        child.neutralOutput();
    }

    @Override
    public void setVelocity(LinearVelocity setpoint) {
        child.setVelocity(RotationsPerSecond.of(setpoint.in(MetersPerSecond) / circumference.in(Meters)));
    }

    @Override
    public LinearVelocity getCurrentVelocity() {
        return MetersPerSecond.of(child.getCurrentVelocity().in(RotationsPerSecond) * circumference.in(Meters));
    }

}
