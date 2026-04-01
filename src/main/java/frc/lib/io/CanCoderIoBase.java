package frc.lib.io;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.component.AngleSensor;
import frc.lib.logging.LoggableIo;

public class CanCoderIoBase extends LoggableIo<CanCoderIoBase.CanCoderIoInputs> implements AngleSensor {
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
