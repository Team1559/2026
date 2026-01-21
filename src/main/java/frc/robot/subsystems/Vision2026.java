package frc.robot.subsystems;

import java.util.function.Supplier;


import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystems.swerve.SwerveDrive;
import frc.lib.subsystems.vision.LimelightCameraIo;
import frc.lib.subsystems.vision.Vision;
import frc.lib.subsystems.vision.VisionCameraIo;

public class Vision2026 extends Vision {
    
    public Vision2026(SwerveDrive drivetrain){
        super("Vision", drivetrain, createCameras(drivetrain));
    }
    
    private static VisionCameraIo[] createCameras(SwerveDrive drivetrain){
        Supplier<Rotation2d> yaw = () -> drivetrain.getPosition().getRotation();
        VisionCameraIo frontRight = new LimelightCameraIo("FrontRight", "limelight-frontr", yaw);
        VisionCameraIo frontLeft = new LimelightCameraIo("FrontLeft", "limelight-frontl", yaw);

        return new VisionCameraIo[]{frontRight, frontLeft};
    }
    
}
