package frc.lib.io;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Temperature;
import org.wpilib.system.Timer;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.component.SwerveModule;
import frc.lib.logging.LoggableIo;

public class SdsSwerveModuleIoBase extends LoggableIo<SdsSwerveModuleIoBase.SwerveInputs> implements SwerveModule {
    @AutoLog
    public static abstract class SwerveInputs implements LoggableInputs {
        public LinearVelocity speed = MetersPerSecond.zero();
        public Rotation2d angle = Rotation2d.ZERO;
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
