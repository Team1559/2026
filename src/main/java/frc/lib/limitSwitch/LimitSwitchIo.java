package frc.lib.limitSwitch;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.loggable.LoggableIo;

public class LimitSwitchIo extends LoggableIo<LimitSwitchIo.LimitSwitchIoInputs> implements BooleanComponent {

    @AutoLog
    public static abstract class LimitSwitchIoInputs implements LoggableInputs{
        boolean atLimit = false;
    }

    public final DigitalInput limitSwitch;

    public LimitSwitchIo(String name, DigitalInput limitSwitch){
        super(name, new LimitSwitchIoInputsAutoLogged());
        this.limitSwitch = limitSwitch;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public boolean get() {
        return getInputs().atLimit;
    }
}
