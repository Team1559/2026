package frc.lib.angularPosition;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.LoggableIo;

public class CanCoderIo extends LoggableIo<CanCoderIo.CanCoderIoInputs> implements AngularPositionSensor{

    @AutoLog
    public static abstract class CanCoderIoInputs implements LoggableInputs {
        public Angle currentAngle = Rotations.of(0);
    }

    private CANcoder canCoder;
    private final StatusSignal<Angle> angle;
    private final Angle offset;

    public CanCoderIo(String name, CANcoder canCoder, Angle offset, CANcoderConfiguration config){
        super(name, new CanCoderIoInputsAutoLogged());
        this.canCoder = canCoder;
        canCoder.getConfigurator().apply(config);
        this.offset = offset;
        angle = canCoder.getAbsolutePosition();
    }

    @Override
    public Angle getAngle() {
        return getInputs().currentAngle;
    }

    @Override
    protected void updateInputs(CanCoderIoInputs inputs) {
        angle.refresh();
        inputs.currentAngle = angle.getValue().minus(offset);
    }
}