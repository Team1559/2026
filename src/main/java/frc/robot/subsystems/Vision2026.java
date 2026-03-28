package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.swerve.SwerveDrive;
import frc.lib.vision.LimelightCameraIoBase;
import frc.lib.vision.LimelightCameraIoReal;
import frc.lib.vision.Vision;
import frc.lib.vision.VisionComponent;

public class Vision2026 extends Vision {

    public Vision2026(SwerveDrive drivetrain) {
        super("Vision", drivetrain, AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                createCameras(drivetrain));
    }

    private static Map<String, VisionComponent> createCameras(SwerveDrive drivetrain) {
        Supplier<Rotation2d> yaw = () -> drivetrain.getPosition().getRotation();
        
        VisionComponent frontStraight = makeCamera("limelight-fronts", yaw);
        VisionComponent frontLeft = makeCamera("limelight", yaw);
        VisionComponent backLeft = makeCamera("limelight-backl", yaw);

        return Map.of("FrontStraight", frontStraight, "FrontLeft", frontLeft, "BackLeft", backLeft);
    }

    private static VisionComponent makeCamera(String hostname, Supplier<Rotation2d> yawSupplier) {
        VisionComponent camera;
        if (Logger.hasReplaySource()) {
            camera = new LimelightCameraIoBase();
        } else {
            camera = new LimelightCameraIoReal(hostname, yawSupplier);
        }
        return camera;
    }
}
