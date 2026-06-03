package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.Logger;

import frc.lib.component.AprilTagSensor;
import frc.lib.io.LimelightCameraIoBase;
import frc.lib.io.LimelightCameraIoReal;
import frc.lib.subsystem.SwerveDrive;
import frc.lib.subsystem.Vision;

public class Vision2026 extends Vision {

    public Vision2026(SwerveDrive drivetrain) {
        super("Vision", drivetrain, AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                createCameras(drivetrain));
    }

    private static Map<String, AprilTagSensor> createCameras(SwerveDrive drivetrain) {
        Supplier<Rotation2d> yaw = () -> drivetrain.getPosition().getRotation();
        
        AprilTagSensor frontStraight = makeCamera("limelight-fronts", yaw);
        AprilTagSensor frontLeft = makeCamera("limelight-frontl", yaw);
        AprilTagSensor backLeft = makeCamera("limelight-backl", yaw);

        return Map.of(
            "FrontStraight", frontStraight,
            "FrontLeft", frontLeft,
            "BackLeft", backLeft
        );
    }

    private static AprilTagSensor makeCamera(String hostname, Supplier<Rotation2d> yawSupplier) {
        AprilTagSensor camera;
        if (Logger.hasReplaySource()) {
            camera = new LimelightCameraIoBase();
        } else {
            camera = new LimelightCameraIoReal(hostname, yawSupplier);
        }
        return camera;
    }
}
