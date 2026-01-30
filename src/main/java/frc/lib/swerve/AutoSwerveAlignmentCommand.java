package frc.lib.swerve;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.lib.DriverAssist;
import frc.lib.swerve.SwerveDrive.SwerveConstraints;

public class AutoSwerveAlignmentCommand extends DeferredCommand {
    public AutoSwerveAlignmentCommand(SwerveDrive drivetrain, SwerveConstraints swerveConstraints,
            DriverAssist driverAssist, Pose2d endPoint, Rotation2d endVelocityDirection) {
        super(() -> getCommand(drivetrain, swerveConstraints, driverAssist, endPoint, endVelocityDirection),
                Set.of(driverAssist, drivetrain));
    }

    private static Command getCommand(SwerveDrive drivetrain, SwerveConstraints swerveConstraints,
            DriverAssist driverAssist, Pose2d endPoint, Rotation2d endVelocityDirection) {

        Translation2d currentTranslation = drivetrain.getPosition().getTranslation();
        Rotation2d targetAngle = endPoint.getTranslation().minus(currentTranslation).getAngle();
        Pose2d startCondition = new Pose2d(currentTranslation, targetAngle);

        Pose2d endCondition = new Pose2d(endPoint.getTranslation(), endVelocityDirection);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startCondition, endCondition);

        PathConstraints constraints = new PathConstraints(swerveConstraints.getMaxLinearVelocity(),
                swerveConstraints.getMaxLinerAccel(), swerveConstraints.getMaxAngularVelocity(),
                swerveConstraints.getMaxAngularAccel());

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null,
                new GoalEndState(0, endPoint.getRotation()));

        path.preventFlipping = true;

        Command command = AutoBuilder.followPath(path);

        driverAssist.configure(command);

        return command;
    }
}