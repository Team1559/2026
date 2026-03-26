package frc.lib.angular_position;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;

public class CanCoderIo extends CanCoderReplayIo{

    private final CANcoder canCoder;
    private final StatusSignal<Angle> angle;

    public CanCoderIo(String name, CANcoder canCoder, Angle offset, CANcoderConfiguration config){
        super(name, offset);
        this.canCoder = canCoder;
        canCoder.getConfigurator().apply(config);
        angle = canCoder.getAbsolutePosition();
    }


    @Override
    protected void updateInputs(CanCoderIoInputs inputs) {
        angle.refresh();
        inputs.rawAngle = angle.getValue();
        logger().debug("CurrentAngle", getAngle());
    }
}
