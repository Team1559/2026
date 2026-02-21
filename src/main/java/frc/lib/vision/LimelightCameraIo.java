package frc.lib.vision;

import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;

public class LimelightCameraIo extends VisionCameraIo {
    private final Supplier<Rotation2d> yaw;
    private final DoubleArrayPublisher orientationPublisher;
    private final DoubleSubscriber latencySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;

    public LimelightCameraIo(String ioName, String hostName, Supplier<Rotation2d> yaw) {
        super(ioName);
        this.yaw = yaw;
        NetworkTable table = NetworkTableInstance.getDefault().getTable(hostName);
        orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
        megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
    }

    @Override
    protected void updateInputs(VisionInputs inputs) {
        // Update connection status based on whether an update has been seen in the last
        // 250ms
        inputs.isConnected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        // Update orientation for MegaTag 2
        orientationPublisher.accept(new double[] { yaw.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0 });
        NetworkTableInstance.getDefault().flush(); // Increases network traffic but recommended by Limelight

        // Read new pose observations from NetworkTables
        Set<Integer> tagIds = new LinkedHashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();

        for (TimestampedDoubleArray rawSample : megatag1Subscriber.readQueue()) {
            PoseObservation poseObservation = getPoseObservation(rawSample, PoseObservationType.MEGATAG_1);
            if (poseObservation != null) {
                poseObservations.add(poseObservation);

                for (int i = 11; i < rawSample.value.length; i += 7) {
                    tagIds.add((int) rawSample.value[i]);
                }
            }
        }

        for (TimestampedDoubleArray rawSample : megatag2Subscriber.readQueue()) {
            PoseObservation poseObservation = getPoseObservation(rawSample, PoseObservationType.MEGATAG_2);
            if (poseObservation != null) {
                poseObservations.add(poseObservation);

                for (int i = 11; i < rawSample.value.length; i += 7) {
                    tagIds.add((int) rawSample.value[i]);
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = poseObservations.toArray(PoseObservation[]::new);

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }

    /** Parses the 3D pose from a Limelight botpose array. */
    private static Pose3d parsePose(double[] rawLLArray) {
        return new Pose3d(
                rawLLArray[0],
                rawLLArray[1],
                rawLLArray[2],
                new Rotation3d(
                        Units.degreesToRadians(rawLLArray[3]),
                        Units.degreesToRadians(rawLLArray[4]),
                        Units.degreesToRadians(rawLLArray[5])));
    }

    private PoseObservation getPoseObservation(TimestampedDoubleArray rawSample, PoseObservationType type) {
        if (rawSample.value.length == 0) {
            return null;
        }
        return new PoseObservation(
                rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
                parsePose(rawSample.value),
                (rawSample.value.length >= 18 && type == PoseObservationType.MEGATAG_1) ? rawSample.value[17] : 0.0,
                (int) rawSample.value[7],
                rawSample.value[9],
                PoseObservationType.MEGATAG_1);
    }
}
