package frc.lib.vision;

import frc.lib.LoggableComponent;
import frc.lib.vision.VisionCameraIo.PoseObservation;

public interface VisionComponent extends LoggableComponent {
    
    int[] getTagIds();

    PoseObservation[] getPoseObservations();

    boolean isConnected();

}
