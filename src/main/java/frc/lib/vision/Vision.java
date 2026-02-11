package frc.lib.vision;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.LoggableSubsystem;
import frc.lib.vision.VisionCameraIo.PoseObservation;
import frc.lib.vision.VisionCameraIo.PoseObservationType;

public class Vision extends LoggableSubsystem {
    private static final double linearStdDevBaseline = 1; // TODO: set constants
    private static final double angularStdDevBaseline = 1;
    private static final double linearStdDevMegatag2Factor = 1;
    private static final double angularStdDevMegatag2Factor = 1;
    private final VisionComponent[] cameras;
    private final VisionConsumer visionConsumer;
    private final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

    public Vision(String name, VisionConsumer visionConsumer, VisionComponent... cameras) {
        super(name);
        this.visionConsumer = visionConsumer;
        this.cameras = cameras;
        addChildren("Cameras", cameras);
    }

    @Override
    public void periodic() {
        super.periodic();
        boolean robotHasPose = false;

        List<Pose3d> tagPoses = new LinkedList<>();
        List<Pose3d> robotPoses = new LinkedList<>();
        List<Pose3d> robotPosesAccepted = new LinkedList<>();
        List<Pose3d> robotPosesRejected = new LinkedList<>();

        for (VisionComponent cam : cameras) {

            for (int tagId : cam.getTagIds()) {
                Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            for (PoseObservation observation : cam.getPoseObservations()) {
                boolean rejectPose = false; // TODO: reject pose
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                if (rejectPose) {
                    continue;
                }

                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;

                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                visionConsumer.addVisionMeasurement(cam.getPose(), cam.getTimestamp(),
                        VecBuilder.fill(cam.getStdevX(), cam.getStdevY(),
                                cam.getStdevRotation().getRadians()));
                robotHasPose = true;

                Logger.recordOutput(getOutputLogPath("TagPoses"), tagPoses.toArray(new Pose3d[0]));
                Logger.recordOutput(getOutputLogPath("RobotPoses"), robotPoses.toArray(new Pose3d[0]));
                Logger.recordOutput(getOutputLogPath("RobotPosesAccepted"), robotPosesAccepted.toArray(new Pose3d[0]));
                Logger.recordOutput(getOutputLogPath("RobotPosesRejected"), robotPosesRejected.toArray(new Pose3d[0]));
            }
        }
        Logger.recordOutput(getOutputLogPath("HasVisionRead"), robotHasPose);
    }
}
