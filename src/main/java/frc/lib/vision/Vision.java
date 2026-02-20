package frc.lib.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.LoggableSubsystem;
import frc.lib.vision.VisionCameraIo.PoseObservation;
import frc.lib.vision.VisionCameraIo.PoseObservationType;

public class Vision extends LoggableSubsystem {
    private static final double linearStdDevBaseline = 0.2;
    private static final double angularStdDevBaseline = 0.2;
    private static final double linearStdDevMegatag2Factor = 1;
    private static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
    private static final double maxAmbiguity = 0.1;
    private static final Distance maxZError = Inches.of(3);
    private final VisionComponent[] cameras;
    private final VisionConsumer visionConsumer;
    private final AprilTagFieldLayout aprilTagLayout;

    public Vision(String name, VisionConsumer visionConsumer, AprilTagFieldLayout aprilTagLayout,
            VisionComponent... cameras) {
        super(name);
        this.visionConsumer = visionConsumer;
        this.aprilTagLayout = aprilTagLayout;
        this.cameras = cameras;
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
                    rejectedObservations.add(new RejectedObservation(observation, "No tags"));
                    continue;
                }

                if (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) {
                    rejectedObservations.add(new RejectedObservation(observation, "One tag, too ambiguous"));
                    continue;
                }

                if (observation.pose().getZ() > maxZError.in(Meters)) {
                    rejectedObservations.add(new RejectedObservation(observation, "Large Z error"));
                    continue;
                }

                if (observation.type() == PoseObservationType.MEGATAG_2 && DriverStation.isDisabled()) {
                    rejectedObservations.add(new RejectedObservation(observation, "Megatag 2 and disabled"));
                    continue;
                }

                if (observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth()) {
                    rejectedObservations.add(new RejectedObservation(observation, "Outside of field"));
                    continue;
                }

                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;

                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }

                acceptedObservations.add(new AcceptedObservation(observation, Meters.of(linearStdDev), Radians.of(angularStdDev)));

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

        Logger.recordOutput(getOutputLogPath("HasVisionRead"), robotHasPose);
        Logger.recordOutput(getOutputLogPath("TagPoses"), tagPoses.toArray(Pose3d[]::new));
        Logger.recordOutput(getOutputLogPath("RobotPosesAccepted"), robotPosesAccepted.toArray(Pose3d[]::new));
        Logger.recordOutput(getOutputLogPath("RobotPosesRejected"), robotPosesRejected.toArray(Pose3d[]::new));

        Logger.recordOutput(getOutputLogPath("RejectedObservations"),
                rejectedObservations.toArray(RejectedObservation[]::new));
        Logger.recordOutput(getOutputLogPath("AcceptedObservations"),
                acceptedObservations.toArray(AcceptedObservation[]::new));

    }

    public record RejectedObservation(
            PoseObservation observation,
            String reason) {}

    public record AcceptedObservation(
            PoseObservation observation,
            Distance linearStdDev,
            Angle angularStdDev) {}
}
