package frc.lib.angular_position;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.measure.Angle;
import frc.lib.logging.LoggableIo;

public class CanCoderIoBase extends LoggableIo<CanCoderIoBase.CanCoderIoInputs> implements AngularPositionSensor {
    @AutoLog
    public static abstract class CanCoderIoInputs implements LoggableInputs {
        public Angle rawAngle = Rotations.of(0);
    }

    public CanCoderIoBase() {
        super(new CanCoderIoInputsAutoLogged());
    }

    @Override
    public Angle getAngle() {
        return getInputs().rawAngle;
    }

}
