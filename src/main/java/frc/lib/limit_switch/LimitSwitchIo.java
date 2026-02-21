package frc.lib.limit_switch;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.loggable.LoggableIo;

public class LimitSwitchIo extends LoggableIo<LimitSwitchIo.LimitSwitchIoInputs> implements BooleanComponent {

    @AutoLog
    public static abstract class LimitSwitchIoInputs implements LoggableInputs{
        boolean atLimit = false;
    }

    private final DigitalInput limitSwitch;

    public LimitSwitchIo(String name, DigitalInput limitSwitch){
        super(name, new LimitSwitchIoInputsAutoLogged());
        this.limitSwitch = limitSwitch;
    }

    @Override
    public boolean getAsBoolean() {
        return getInputs().atLimit;
    }

    @Override
    protected void updateInputs(LimitSwitchIoInputs inputs) {
        inputs.atLimit = limitSwitch.get();
    }
}
