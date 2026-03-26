package frc.lib.limit_switch;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.logging.LoggableIo;

public class LimitSwitchReplayIo extends LoggableIo<LimitSwitchReplayIo.LimitSwitchIoInputs> implements BooleanComponent {
    @AutoLog
    public static abstract class LimitSwitchIoInputs implements LoggableInputs {
        boolean atLimit = false;
    }

    public LimitSwitchReplayIo(String name) {
        super(name, new LimitSwitchIoInputsAutoLogged());
    }

    @Override
    public boolean getAsBoolean() {
        return getInputs().atLimit;
    }

}
