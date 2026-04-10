package frc.lib.component;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

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

    default void setState(SwerveModuleState state) {
        setAngle(state.angle);
        setSpeed(MetersPerSecond.of(state.speedMetersPerSecond));
    }

    LinearVelocity getSpeed();

    Rotation2d getAngle();

    Distance getDistance();
}
