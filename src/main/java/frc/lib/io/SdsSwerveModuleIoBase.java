package frc.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.component.SwerveModule;
import frc.lib.logging.LoggableIo;

public class SdsSwerveModuleIoBase extends LoggableIo<SdsSwerveModuleIoBase.SwerveInputs> implements SwerveModule {
    @AutoLog
    public static abstract class SwerveInputs implements LoggableInputs {
        public LinearVelocity speed = MetersPerSecond.zero();
        public Rotation2d angle = Rotation2d.kZero;
        public Distance distance = Meters.of(0);
        public Temperature steerMotorTemp = Celsius.of(20);
        public Temperature driveMotorTemp = Celsius.of(20);
        public Current steerMotorCurrent = Amps.of(0);
        public Current driveMotorCurrent = Amps.of(0);
    }

    private final Translation2d location;

    public SdsSwerveModuleIoBase(Translation2d location) {
        super(new SwerveInputsAutoLogged());
        this.location = location;
    }

    @Override
    public void setSpeed(LinearVelocity speed) {
        logger().debug("Speed", speed);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        logger().debug("Angle", angle)
                .debug("Timestamp", Timer.getTimestamp());
    }

    @Override
    public Translation2d getLocation() {
        return location;
    }

    @Override
    public LinearVelocity getSpeed() {
        return getInputs().speed;
    }

    @Override
    public Rotation2d getAngle() {
        return getInputs().angle;
    }

    @Override
    public Distance getDistance() {
        return getInputs().distance;
    }

    @Override
    public void neutralOutput() {
        //Explcitly here for the purpose of doing nothing, for logging purposes.
    }
}
