package frc.lib.angular_position;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.measure.Angle;
import frc.lib.logging.LoggableIo;

public class CanCoderReplayIo extends LoggableIo<CanCoderReplayIo.CanCoderIoInputs> implements AngularPositionSensor {
    @AutoLog
    public static abstract class CanCoderIoInputs implements LoggableInputs {
        public Angle rawAngle = Rotations.of(0);
    }

    private final Angle offset;

    public CanCoderReplayIo(String name, Angle offset) {
        super(name, new CanCoderIoInputsAutoLogged());
        this.offset = offset;
    }

    @Override
    public Angle getAngle() {
        return getInputs().rawAngle.minus(offset);
    }

}
