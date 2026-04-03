package frc.lib.intermediate;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import frc.lib.component.AngularVelocityComponent;
import frc.lib.component.LinearVelocityComponent;
import frc.lib.logging.LoggableIntermediate;

public class WheelVelocityIntermediate extends LoggableIntermediate implements LinearVelocityComponent {

    private AngularVelocityComponent wheel;
    private Distance circumfrence;

    public WheelVelocityIntermediate(AngularVelocityComponent wheel, Distance wheelRadius) {
        this.wheel = wheel;
        circumfrence = wheelRadius.times(2 * Math.PI);
    }

    @Override
    public void neutralOutput() {
        wheel.neutralOutput();
    }

    @Override
    public void setVelocity(LinearVelocity setpoint) {
        wheel.setVelocity(RotationsPerSecond.of(setpoint.in(MetersPerSecond) / circumfrence.in(Meters)));
    }

    @Override
    public LinearVelocity getCurrentVelocity() {
        return MetersPerSecond.of(wheel.getCurrentVelocity().in(RotationsPerSecond) * circumfrence.in(Meters));
    }

}
