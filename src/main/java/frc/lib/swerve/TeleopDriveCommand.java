package frc.lib.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.swerve.SwerveDrive.SwerveConstraints;

public class TeleopDriveCommand extends Command {
    private static final double DEADBAND = .10;

    private final DoubleSupplier ySupplier;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier rSupplier;

    private final double maxLinearVelocity;
    private final double maxRotationalVelocity;

    private final SwerveDrive swerveDrive;

    private final BooleanSupplier driveRobotOriented;

    public TeleopDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier,
            SwerveConstraints swerveConstraints, SwerveDrive swerveDrive) {
        this(xSupplier, ySupplier, rSupplier, swerveConstraints, swerveDrive, () -> false);
    }

    public TeleopDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier,
            SwerveConstraints swerveConstraints, SwerveDrive swerveDrive, BooleanSupplier driveRobotOriented) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        this.maxLinearVelocity = swerveConstraints.getMaxLinearVelocity();
        this.maxRotationalVelocity = swerveConstraints.getMaxAngularVelocity();

        this.swerveDrive = swerveDrive;

        this.driveRobotOriented = driveRobotOriented;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        double y = ySupplier.getAsDouble();
        double rotation = rSupplier.getAsDouble();

        double magnitude = Math.hypot(x, y);

        if (magnitude > 1) {
            x /= magnitude;
            y /= magnitude;
        } else if (magnitude < DEADBAND) {
            x = 0;
            y = 0;
        } else {
            double deadbandMagnitude = MathUtil.applyDeadband(magnitude, DEADBAND);
            x *= deadbandMagnitude / magnitude;
            y *= deadbandMagnitude / magnitude;
        }

        rotation = MathUtil.applyDeadband(rotation, DEADBAND);

        x *= maxLinearVelocity;
        y *= maxLinearVelocity;
        rotation *= maxRotationalVelocity;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(x, y, rotation);

        if (driveRobotOriented.getAsBoolean()) {
            swerveDrive.driveRobotOriented(chassisSpeeds);
        } else if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            swerveDrive.driveFieldOriented(new ChassisSpeeds(-x, -y, rotation));
        } else {
            swerveDrive.driveFieldOriented(chassisSpeeds);
        }
    }
}