package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;
import org.wpilib.fields.FieldTag;
import org.wpilib.fields.Fields;
import org.wpilib.fields.Field;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;

import frc.lib.component.AprilTagSensor;
import frc.lib.io.LimelightCameraIoBase;
import frc.lib.io.LimelightCameraIoReal;
import frc.lib.subsystem.SwerveDrive;
import frc.lib.subsystem.Vision;

public class Vision2026 extends Vision {

    public Vision2026(SwerveDrive drivetrain) {
        super("Vision", drivetrain, createCameras(drivetrain), Fields.FRC_2026_REBUILT_ANDY_MARK.loadField());
    }

    private static Map<String, AprilTagSensor> createCameras(SwerveDrive drivetrain) {
        Supplier<Rotation2d> yaw = () -> drivetrain.getPosition().getRotation();

        AprilTagSensor frontStraight = makeCamera("limelight-fronts", yaw);
        AprilTagSensor frontLeft = makeCamera("limelight-frontl", yaw);
        AprilTagSensor backLeft = makeCamera("limelight-backl", yaw);

        return Map.of(
                "FrontStraight", frontStraight,
                "FrontLeft", frontLeft,
                "BackLeft", backLeft);
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
