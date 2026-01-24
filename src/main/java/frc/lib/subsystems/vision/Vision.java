package frc.lib.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import frc.lib.subsystems.LoggableSubsystem;
import frc.lib.subsystems.vision.VisionCameraIo.VisionInputs;

public class Vision extends LoggableSubsystem {
    private final VisionComponent[] cameras;
    private final VisionConsumer visionConsumer;

    public Vision(String name, VisionConsumer visionConsumer, VisionComponent ... cameras) {
        super(name);
        this.visionConsumer = visionConsumer;
        this.cameras = cameras;
        addChildren("Cameras", cameras);
    }

    @Override
    public void periodic() {
        super.periodic();
        boolean robotHasPose = false;
        for (VisionComponent cam : cameras) {
            if (cam.hasPose()) {
                visionConsumer.addVisionMeasurement(cam.getPose(), cam.getTimestamp(),
                        VecBuilder.fill(cam.getStdevX(), cam.getStdevY(), cam.getStdevRotation().getRadians()));
                robotHasPose = true;
            }
        }
        Logger.recordOutput(getOutputLogPath("HasVisionRead"), robotHasPose);
    }
}
