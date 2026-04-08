package frc.lib.intermediate;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

import frc.lib.component.AngleComponent;
import frc.lib.component.AngleSensor;
import frc.lib.logging.LoggableIntermediate;

public class FusedAngleIntermediate extends LoggableIntermediate implements AngleComponent{
    private final AngleSensor sensor;
    private final AngleComponent actuator;
    private Angle sensorOffset = Degrees.zero();

    public FusedAngleIntermediate(AngleSensor sensor, AngleComponent actuator) {
        this.sensor = sensor;
        this.actuator = actuator;

        this.addChild("Sensor", sensor);
        this.addChild("Actuator", actuator);
    }

    @Override
    public Angle getAngle() {
        return sensor.getAngle().minus(sensorOffset);
    }

    @Override
    public void neutralOutput() {
       actuator.neutralOutput();
    }

    @Override
    public void setAngle(Angle angle) {
        actuator.setAngle(actuator.getAngle().plus(angle.plus(sensorOffset)));
    }

    @Override
    public void setPercievedAngle(Angle angle) {
        sensorOffset = getAngle().minus(angle);
    }
}
