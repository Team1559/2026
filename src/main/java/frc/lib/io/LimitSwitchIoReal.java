package frc.lib.io;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchIoReal extends LimitSwitchIoBase {

    private final DigitalInput limitSwitch;

    public LimitSwitchIoReal(DigitalInput limitSwitch){
        this.limitSwitch = limitSwitch;
    }

    @Override
    protected void updateInputs(LimitSwitchIoInputs inputs) {
        inputs.atLimit = limitSwitch.get();
    }
}
