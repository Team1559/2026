package frc.lib.limit_switch;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchIo extends LimitSwitchReplayIo {

    private final DigitalInput limitSwitch;

    public LimitSwitchIo(String name, DigitalInput limitSwitch){
        super(name);
        this.limitSwitch = limitSwitch;
    }

    @Override
    protected void updateInputs(LimitSwitchIoInputs inputs) {
        inputs.atLimit = limitSwitch.get();
    }
}
