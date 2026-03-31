package frc.lib.limit_switch;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.logging.LoggableIo;

public class LimitSwitchIoBase extends LoggableIo<LimitSwitchIoBase.LimitSwitchIoInputs> implements BooleanComponent {
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
