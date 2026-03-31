package frc.lib.io;

import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

public class CanCoderIoReal extends CanCoderIoBase{

    @SuppressWarnings("unused") // To avoid garbage collection of canCoder
    private final CANcoder canCoder;
    private final StatusSignal<Angle> angle;

    public CanCoderIoReal(CANcoder canCoder, CANcoderConfiguration config) {
        this.canCoder = canCoder;
        canCoder.getConfigurator().apply(config);
        angle = canCoder.getAbsolutePosition();
    }


    @Override
    protected void updateInputs(CanCoderIoInputs inputs) {
        angle.refresh();
        inputs.rawAngle = angle.getValue();
    }
}
