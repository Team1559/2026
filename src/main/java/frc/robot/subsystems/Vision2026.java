package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.swerve.SwerveDrive;
import frc.lib.vision.LimelightCameraIo;
import frc.lib.vision.Vision;
import frc.lib.vision.VisionCameraIo;


public class Vision2026 extends Vision {
    
    public Vision2026(SwerveDrive drivetrain){
        super("Vision", drivetrain, AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark), createCameras(drivetrain));
    }
    
    private static VisionCameraIo[] createCameras(SwerveDrive drivetrain){
        Supplier<Rotation2d> yaw = () -> drivetrain.getPosition().getRotation();
        VisionCameraIo frontStraight = new LimelightCameraIo("FrontStraight", "limelight-fronts", yaw);
        VisionCameraIo frontLeft = new LimelightCameraIo("FrontLeft", "limelight", yaw);
        VisionCameraIo backLeft = new LimelightCameraIo("BackLeft", "limelight-backl", yaw);


        return new VisionCameraIo[]{frontStraight, frontLeft, backLeft};
    }
    
}
