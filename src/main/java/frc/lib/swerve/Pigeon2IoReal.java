package frc.lib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.lib.CanRefreshRate;

public class Pigeon2IoReal extends Pigeon2IoBase {

    @SuppressWarnings("unused") // Needed to prevent garbage collection
    private final Pigeon2 gyro;

    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<Angle> yaw;

    public Pigeon2IoReal(Pigeon2 gyro) {
        this.gyro = gyro;
        yaw = gyro.getYaw();
        pitch = gyro.getPitch();
        roll = gyro.getRoll();

        yaw.setUpdateFrequency(CanRefreshRate.FAST.rateHz);
        pitch.setUpdateFrequency(CanRefreshRate.SLOW.rateHz);
        roll.setUpdateFrequency(CanRefreshRate.SLOW.rateHz);
    }

    @Override
    protected void updateInputs(GyroInputs inputs) {
        BaseStatusSignal.refreshAll(roll, pitch, yaw);
        inputs.pitch = Rotation2d.fromDegrees(pitch.getValueAsDouble());
        inputs.roll = Rotation2d.fromDegrees(roll.getValueAsDouble());
        inputs.yaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    }
}
