package frc.lib.component;

import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import frc.lib.logging.LoggableComponent;
import frc.lib.util.NeutralOutput;

public interface SwerveModule extends LoggableComponent, NeutralOutput {

    /**
     * Sets the speed of the wheel.
     * 
     * @param speed the desired wheel speed in meters per second
     */
    void setSpeed(LinearVelocity speed);

    /**
     * Sets the target direction for the wheel
     * 
     * @param angle the desired angle of the wheel
     */
    void setAngle(Rotation2d angle);

    /**
     * @return The location of the wheel relative to the origin of the robot in
     *         meters
     */
    Translation2d getLocation();

    default void setState(SwerveModuleVelocity state) {
        setAngle(state.angle);
        setSpeed(MetersPerSecond.of(state.velocity));
    }

    LinearVelocity getSpeed();

    Rotation2d getAngle();

    Distance getDistance();
}
