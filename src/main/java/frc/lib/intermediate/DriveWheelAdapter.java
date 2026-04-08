package frc.lib.intermediate;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import frc.lib.component.AngleSensor;
import frc.lib.component.AngularVelocityComponent;
import frc.lib.component.DistanceSensor;
import frc.lib.component.LinearVelocityComponent;
import frc.lib.logging.LoggableIntermediate;

public class DriveWheelAdapter<T extends AngleSensor & AngularVelocityComponent> extends LoggableIntermediate
        implements DistanceSensor, LinearVelocityComponent {
    private final T motor;
    private final Distance circumference;
    private final double reductionRatio;

    public DriveWheelAdapter(T motor, Distance wheelRadius, double reductionRatio) {
        this.motor = motor;
        this.setChild(motor);
        this.reductionRatio = reductionRatio;

        circumference = wheelRadius.times(2 * Math.PI);
    }

    @Override
    public Distance getDistance() {
        return circumference.times(motor.getAngle().in(Rotations)).div(reductionRatio);
    }

    @Override
    public void neutralOutput() {
        motor.neutralOutput();
    }

    @Override
    public void setVelocity(LinearVelocity setpoint) {
        motor.setVelocity(
                RotationsPerSecond.of(setpoint.in(MetersPerSecond) / circumference.in(Meters)).times(reductionRatio));
    }

    @Override
    public LinearVelocity getCurrentVelocity() {
        return MetersPerSecond.of(motor.getCurrentVelocity().in(RotationsPerSecond) * circumference.in(Meters))
                .div(reductionRatio);
    }

}
