package frc.lib.intermediate;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import frc.lib.component.AngleSensor;
import frc.lib.component.AngularVelocityComponent;
import frc.lib.component.DistanceSensor;
import frc.lib.component.LinearVelocityComponent;
import frc.lib.logging.LoggableAdapter;

public class DriveWheelAdapter<T extends AngleSensor & AngularVelocityComponent> extends LoggableAdapter<T>
        implements DistanceSensor, LinearVelocityComponent {
    private final Distance circumference;
    private final double reductionRatio;

    public DriveWheelAdapter(T motor, Distance wheelRadius, double reductionRatio) {
        super(motor);
        this.reductionRatio = reductionRatio;

        circumference = wheelRadius.times(2 * Math.PI);
    }

    @Override
    public Distance getDistance() {
        return circumference.times(child.getAngle().in(Rotations)).div(reductionRatio);
    }

    @Override
    public void neutralOutput() {
        child.neutralOutput();
    }

    @Override
    public void setVelocity(LinearVelocity setpoint) {
        child.setVelocity(
                RotationsPerSecond.of(setpoint.in(MetersPerSecond) / circumference.in(Meters)).times(reductionRatio));
    }

    @Override
    public LinearVelocity getCurrentVelocity() {
        return MetersPerSecond.of(child.getCurrentVelocity().in(RotationsPerSecond) * circumference.in(Meters))
                .div(reductionRatio);
    }

}
