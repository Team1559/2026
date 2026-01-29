package frc.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.LoggableComponent;

public interface SwerveModule extends LoggableComponent {

    /**
     * Sets the speed of the wheel.
     * 
     * @param speed the desired wheel speed in meters per second
     */
    void setSpeed(double speed);

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
        setSpeed(state.speedMetersPerSecond);
    }

    double getSpeed();

    Rotation2d getAngle();

    double getDistance();

    double getSteerMotorTemp();

    double getDriveMotorTemp();

    double getSteerMotorCurrent();

    double getDriveMotorCurrent();
}
