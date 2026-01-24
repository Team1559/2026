package frc.lib.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class Pigeon2Io extends GyroIo {

    @SuppressWarnings("unused") // Needed to prevent garbage collection
    private final Pigeon2 gyro;

    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<Angle> yaw;

    public Pigeon2Io(String ioName, Pigeon2 gyro) {
        super(ioName);
        this.gyro = gyro;
        yaw = gyro.getYaw();
        pitch = gyro.getPitch();
        roll = gyro.getRoll();

        yaw.setUpdateFrequency(100);
        pitch.setUpdateFrequency(100);
        roll.setUpdateFrequency(100);
    }

    @Override
    protected void updateInputs(GyroInputs inputs) {
        StatusSignal.refreshAll(roll, pitch, yaw);
        inputs.pitch = Rotation2d.fromDegrees(pitch.getValueAsDouble());
        inputs.roll = Rotation2d.fromDegrees(roll.getValueAsDouble());
        inputs.yaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    }
}
