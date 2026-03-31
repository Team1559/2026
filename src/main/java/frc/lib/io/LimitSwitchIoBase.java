package frc.lib.io;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.component.BooleanSensor;
import frc.lib.limit_switch.LimitSwitchIoInputsAutoLogged;
import frc.lib.logging.LoggableIo;

public class LimitSwitchIoBase extends LoggableIo<LimitSwitchIoBase.LimitSwitchIoInputs> implements BooleanSensor {
    @AutoLog
    public static abstract class LimitSwitchIoInputs implements LoggableInputs {
        public boolean atLimit = false;
    }

    public LimitSwitchIoBase() {
        super(new LimitSwitchIoInputsAutoLogged());
    }

    @Override
    public boolean getAsBoolean() {
        return getInputs().atLimit;
    }

}
