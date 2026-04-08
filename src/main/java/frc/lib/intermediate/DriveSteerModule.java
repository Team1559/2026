package frc.lib.intermediate;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import frc.lib.component.AngleComponent;
import frc.lib.component.AngleSensor;
import frc.lib.component.SwerveModule;
import frc.lib.logging.LoggableIntermediate;

public class DriveSteerModule extends LoggableIntermediate implements SwerveModule {
    private final AngleComponent steerMotor;
    private final DriveWheelAdapter<?> driveMotor;
    private final AngleSensor encoder;
    private final Translation2d location;

    public DriveSteerModule(Translation2d location, AngleComponent steerMotor,
            DriveWheelAdapter<?> driveMotor,
            AngleSensor cancoder) {

        this.location = location;
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
        this.encoder = cancoder;
        
        addChild("DriveMotor", driveMotor);
        addChild("SteerMotor", steerMotor);
        addChild("Cancoder", encoder);
    }

    @Override
    public void setSpeed(LinearVelocity speed) {
        driveMotor.setVelocity(speed);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        steerMotor.setAngle(angle);
    }

    @Override
    public void neutralOutput() {
        driveMotor.neutralOutput();
        steerMotor.neutralOutput();
    }

    @Override
    public Translation2d getLocation() {
        return location;
    }

    @Override
    public LinearVelocity getSpeed() {
        return driveMotor.getCurrentVelocity();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(steerMotor.getAngle().in(Rotations));
    }

    @Override
    public Distance getDistance() {
        return driveMotor.getDistance();
    }
}
