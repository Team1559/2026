package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;

import org.littletonrobotics.junction.Logger;

import frc.lib.command.LockedHeadingSwerveDriveCommand;
import frc.lib.subsystem.SwerveDrive;
import frc.lib.subsystem.SwerveDrive.SwerveConstraints;

public class NatesNinetyCommand extends DeferredCommand {
    private static final PIDController pid = new PIDController(30, 0, 0);
    static {
        pid.setTolerance(1 / 360.0);
        pid.enableContinuousInput(-1, 1);

    }

    public NatesNinetyCommand(SwerveDrive drivetrain, DoubleSupplier leftStickX, DoubleSupplier leftStickY,
            SwerveConstraints swerveConstraints, double deadband) {
        super(() -> generateCommand(drivetrain, leftStickX, leftStickY, swerveConstraints, deadband),
                Set.of(drivetrain));
    }

    private static LockedHeadingSwerveDriveCommand generateCommand(SwerveDrive drivetrain, DoubleSupplier leftStickX,
            DoubleSupplier leftStickY,
            SwerveConstraints swerveConstraints, double deadband) {
        Rotation2d heading = Rotation2d
                .fromDegrees(Math.round(drivetrain.getPosition().getRotation().getDegrees() / 90.0) * 90.0);
        Logger.recordOutput("Temp/heading", heading);

        return new LockedHeadingSwerveDriveCommand(drivetrain, leftStickX, leftStickY, heading, swerveConstraints, pid,
                deadband);
    }
}
