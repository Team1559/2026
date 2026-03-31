package frc.lib.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.logging.LoggableSubsystem;
import frc.lib.vision.VisionComponent.PoseObservation;
import frc.lib.vision.VisionComponent.PoseObservationType;

public class Vision extends LoggableSubsystem {
    private static final double LINEAR_STD_DEV_BASELINE = 0.4;
    private static final double ANGULAR_STD_DEV_BASELINE = 0.2;
    private static final double LINEAR_STD_DEV_MEGATAG_2_FACTOR = 1;
    private static final double ANGULAR_STD_DEV_MEGATAG_2_FACTOR = Double.POSITIVE_INFINITY;
    private static final double MAX_AMBIGUITY = 0.1;
    private static final Distance MAX_Z_ERROR = Inches.of(3);
    private final VisionComponent[] cameras;
    private final VisionConsumer visionConsumer;
    private final AprilTagFieldLayout aprilTagLayout;

    public Vision(String name, VisionConsumer visionConsumer, AprilTagFieldLayout aprilTagLayout,
            Map<String, VisionComponent> cameras) {
        super(name);
        this.visionConsumer = visionConsumer;
        this.aprilTagLayout = aprilTagLayout;
        this.cameras = cameras.values().toArray(VisionComponent[]::new);
        addChildren("Cameras", cameras);
    }

    @Override
    public void periodic() {
        super.periodic();
        boolean robotHasPose = false;

        List<Pose3d> tagPoses = new LinkedList<>();

        List<RejectedObservation> rejectedObservations = new LinkedList<>();
        List<AcceptedObservation> acceptedObservations = new LinkedList<>();

        for (VisionComponent cam : cameras) {
            for (int tagId : cam.getTagIds()) {
                Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            for (PoseObservation observation : cam.getPoseObservations()) {

                if (observation.tagCount() == 0) {
                    rejectedObservations.add(new RejectedObservation(observation, RejectionReason.NO_TAGS));
                    continue;
                }

                if (observation.tagCount() == 1 && observation.ambiguity() > MAX_AMBIGUITY) {
                    rejectedObservations
                            .add(new RejectedObservation(observation, RejectionReason.ONE_TAG_HIGH_AMBIGUITY));
                    continue;
                }

                if (observation.pose().getZ() > MAX_Z_ERROR.in(Meters)) {
                    rejectedObservations.add(new RejectedObservation(observation, RejectionReason.HIGH_Z_ERROR));
                    continue;
                }

                if (observation.type() == PoseObservationType.MEGATAG_2 && DriverStation.isDisabled()) {
                    rejectedObservations.add(new RejectedObservation(observation, RejectionReason.MT2_AND_DISABLED));
                    continue;
                }

                if (observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth()) {
                    rejectedObservations.add(new RejectedObservation(observation, RejectionReason.OUT_OF_FIELD));
                    continue;
                }

                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
                double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= LINEAR_STD_DEV_MEGATAG_2_FACTOR;
                    angularStdDev *= ANGULAR_STD_DEV_MEGATAG_2_FACTOR;
                }

                acceptedObservations
                        .add(new AcceptedObservation(observation, linearStdDev, angularStdDev));

                visionConsumer.addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
                robotHasPose = true;
            }
        }

        List<Pose3d> robotPosesAccepted = new LinkedList<>();
        List<Pose3d> robotPosesRejected = new LinkedList<>();

        for (RejectedObservation observation : rejectedObservations) {
            robotPosesRejected.add(observation.observation.pose());
        }

        for (AcceptedObservation observation : acceptedObservations) {
            robotPosesAccepted.add(observation.observation.pose());
        }

        logger().dashboard("HasVisionRead", robotHasPose)
                .debug("TagPoses", tagPoses.toArray(Pose3d[]::new))
                .debug("RobotPosesAccepted", robotPosesAccepted.toArray(Pose3d[]::new))
                .debug("RobotPosesRejected", robotPosesRejected.toArray(Pose3d[]::new))
                .debug("RejectedObservations",
                        rejectedObservations.toArray(RejectedObservation[]::new))
                .debug("AcceptedObservations",
                        acceptedObservations.toArray(AcceptedObservation[]::new));

    }

    public enum RejectionReason {
        NO_TAGS,
        ONE_TAG_HIGH_AMBIGUITY,
        HIGH_Z_ERROR,
        MT2_AND_DISABLED,
        OUT_OF_FIELD,
    }

    public record RejectedObservation(
            PoseObservation observation,
            RejectionReason reason) {
    }

    public record AcceptedObservation(
            PoseObservation observation,
            double linearStdDev,
            double angularStdDev) {
    }
}
