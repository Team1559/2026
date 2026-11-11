package frc.lib.intermediate;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

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
