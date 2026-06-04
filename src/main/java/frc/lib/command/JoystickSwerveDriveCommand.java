package frc.lib.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.lib.subsystem.SwerveDrive;
import frc.lib.subsystem.SwerveDrive.SwerveConstraints;

public class JoystickSwerveDriveCommand extends SwerveDriveCommand {
    private final DoubleSupplier leftStickX;
    private final DoubleSupplier leftStickY;
    private final DoubleSupplier rightStickX;
    private final SwerveConstraints swerveConstraints;
    private final double deadband;

    public JoystickSwerveDriveCommand(SwerveDrive swerveDrive,
            DoubleSupplier leftStickX,
            DoubleSupplier leftStickY,
            DoubleSupplier rightStickX,
            SwerveConstraints swerveConstraints,
            double deadband,
            boolean driveRobotOriented) {
        super(swerveDrive, driveRobotOriented);
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickX = rightStickX;
        this.swerveConstraints = swerveConstraints;
        this.deadband = deadband;
    }

    @Override
    protected ChassisSpeeds calculateChassisSpeeds() {
        double x = leftStickY.getAsDouble() * -1;
        double y = leftStickX.getAsDouble() * -1;
        double rotation = rightStickX.getAsDouble() * -1;

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

        rotation = MathUtil.applyDeadband(rotation, deadband);

        return new ChassisSpeeds(swerveConstraints.getMaxLinearVelocity().times(x),
                swerveConstraints.getMaxLinearVelocity().times(y),
                swerveConstraints.getMaxAngularVelocity().times(rotation));
    }
}
