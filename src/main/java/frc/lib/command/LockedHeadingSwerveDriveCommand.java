package frc.lib.command;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.lib.subsystem.SwerveDrive;
import frc.lib.subsystem.SwerveDrive.SwerveConstraints;

public class LockedHeadingSwerveDriveCommand extends SwerveDriveCommand {
    private final DoubleSupplier leftStickX;
    private final DoubleSupplier leftStickY;
    private final SwerveConstraints swerveConstraints;
    private final double deadband;
    private final Rotation2d heading;
    private final PIDController pid;

    public LockedHeadingSwerveDriveCommand(SwerveDrive drivetrain, DoubleSupplier leftStickX, DoubleSupplier leftStickY,
            Rotation2d heading, SwerveConstraints swerveConstraints, PIDController pid, double deadband) {
        super(drivetrain, false);
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.deadband = deadband;
        this.swerveConstraints = swerveConstraints;
        this.heading = heading;
        this.pid = pid;
        pid.setSetpoint(0);
    }

    @Override
    protected ChassisSpeeds calculateChassisSpeeds() {
        double x = leftStickY.getAsDouble() * -1;
        double y = leftStickX.getAsDouble() * -1;

        double magnitude = Math.hypot(x, y);

        if (magnitude > 1) {
            x /= magnitude;
            y /= magnitude;
        } else if (magnitude < deadband) {
            x = 0;
            y = 0;
        } else {
            double deadbandMagnitude = MathUtil.applyDeadband(magnitude, deadband);
            x *= deadbandMagnitude / magnitude;
            y *= deadbandMagnitude / magnitude;
        }

        Angle angleToTarget = heading.minus(swerveDrive.getPosition().getRotation()).getMeasure();

        AngularVelocity angularVelocity = RadiansPerSecond.of(pid.calculate(-angleToTarget.in(Rotations)));

        angularVelocity = RadiansPerSecond.of(MathUtil.clamp(angularVelocity.in(RadiansPerSecond),
                -1 * swerveConstraints.getMaxAngularVelocity().in(RadiansPerSecond),
                swerveConstraints.getMaxAngularVelocity().in(RadiansPerSecond)));

        return new ChassisSpeeds(swerveConstraints.getMaxLinearVelocity().times(x),
                swerveConstraints.getMaxLinearVelocity().times(y), angularVelocity);
    }

}
