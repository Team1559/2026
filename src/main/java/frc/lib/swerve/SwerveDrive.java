package frc.lib.swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LoggableSubsystem;
import frc.lib.vision.VisionConsumer;

public class SwerveDrive extends LoggableSubsystem implements VisionConsumer {
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator estimator;
    private final GyroIo gyro;

    private final Field2d field;

    private double maxLinearAcceleration = Double.POSITIVE_INFINITY;
    private Rotation2d maxRotationalAcceleration = Rotation2d.fromRadians(Double.POSITIVE_INFINITY);

    private ChassisSpeeds targetSpeed = new ChassisSpeeds();

    private final double ROBOT_PERIOD = 0.02;

    public SwerveDrive(String name, GyroIo gyro, SwerveModule... modules) {
        super(name);
        this.gyro = gyro;
        this.modules = modules;

        addChildren("Modules", modules);
        addChildren("Gyro", gyro);

        Translation2d[] locations = new Translation2d[modules.length];
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < locations.length; i++) {
            locations[i] = modules[i].getLocation();
            positions[i] = new SwerveModulePosition(modules[i].getDistance(), modules[i].getAngle());
        }

        this.kinematics = new SwerveDriveKinematics(locations);
        this.estimator = new SwerveDrivePoseEstimator(kinematics, gyro.getInputs().yaw, positions,
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))
                        : new Pose2d()); // Changed the initial rotation

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void configureAuto(RobotConfig robotConfig) {
        AutoBuilder.configure(this::getPosition, this::resetPose, this::getCurrentSpeed, this::driveRobotOriented,
                new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(10)), robotConfig,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this);
    }

    public void driveFieldOriented(ChassisSpeeds speeds) {
        driveRobotOriented(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPosition().getRotation()));
    }

    public void stopDriving() {
        driveRobotOriented(new ChassisSpeeds(0, 0, 0));
    }

    public void driveRobotOriented(ChassisSpeeds speeds) {
        targetSpeed = speeds;
    }

    public Pose2d getPosition() {
        return estimator.getEstimatedPosition();
    }

    protected SwerveModule[] getModules() {
        return modules.clone();
    }

    public ChassisSpeeds getCurrentSpeed() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModuleState(modules[i].getSpeed(), modules[i].getAngle());
        }
        return kinematics.toChassisSpeeds(states);
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = new SwerveModulePosition(modules[i].getDistance(), modules[i].getAngle());
        }
        return positions;
    }

    public void setAccelerationLimits(double maxLinearAcceleration, Rotation2d maxRotationalAcceleration) {
        this.maxLinearAcceleration = maxLinearAcceleration;
        this.maxRotationalAcceleration = maxRotationalAcceleration;
    }

    public void addVisionMeasurement(Pose2d estimatedPose2d, double timestamp, Matrix<N3, N1> standardDeviation) {
        estimator.addVisionMeasurement(estimatedPose2d, timestamp, standardDeviation);
    }

    @Override
    public void periodic() {
        super.periodic();
        drive();
        updateOdometry();
        log();
    }

    public void updateOdometry() {
        estimator.update(gyro.getInputs().yaw, getModulePositions());
    }

    private void drive() {
        ChassisSpeeds currentChassisSpeeds = getCurrentSpeed();

        Translation2d currentLinearVelocity = new Translation2d(currentChassisSpeeds.vxMetersPerSecond,
                currentChassisSpeeds.vyMetersPerSecond);
        Translation2d targetLinearVelocity = new Translation2d(targetSpeed.vxMetersPerSecond,
                targetSpeed.vyMetersPerSecond);
        Translation2d targetLinearAcceleration = targetLinearVelocity.minus(currentLinearVelocity).div(ROBOT_PERIOD);

        if (targetLinearAcceleration.getNorm() > maxLinearAcceleration) {
            targetLinearAcceleration = targetLinearAcceleration.div(targetLinearAcceleration.getNorm())
                    .times(maxLinearAcceleration);
        }

        targetLinearVelocity = currentLinearVelocity.plus(targetLinearAcceleration.times(ROBOT_PERIOD));

        Rotation2d currentRotationalVelocity = Rotation2d.fromRadians(currentChassisSpeeds.omegaRadiansPerSecond);

        Rotation2d targetRotationalVelocity = Rotation2d.fromRadians(targetSpeed.omegaRadiansPerSecond);
        Rotation2d targetRotationalAcceleration = (Rotation2d.fromRadians(targetRotationalVelocity.getRadians() - currentRotationalVelocity.getRadians())) //TODO: debug when chasis is done
        //Use this math instead of WPIs built - in .plus() method, because the .plus() method clamps the output from -pi to pi radians.
                .div(ROBOT_PERIOD);
        if (targetRotationalAcceleration.getRadians() > maxRotationalAcceleration.getRadians()) {
            if (targetRotationalAcceleration.getRadians() > 0) {
                targetRotationalAcceleration = maxRotationalAcceleration;
            } else {
                targetRotationalAcceleration = maxRotationalAcceleration.times(-1);
            }
        }
        targetRotationalVelocity = Rotation2d.fromRadians(currentRotationalVelocity.getRadians() + (targetRotationalAcceleration.getRadians() * ROBOT_PERIOD));
        //Use this math instead of WPIs built - in .plus() method, because the .plus() method clamps the output from -pi to pi radians.

        Logger.recordOutput(getOutputLogPath("TargetLinearVelocity"), targetLinearVelocity);
        Logger.recordOutput(getOutputLogPath("TargetRotationalVelocity"), targetRotationalVelocity);

        ChassisSpeeds accelLimitedSpeeds = new ChassisSpeeds(targetLinearVelocity.getX(), targetLinearVelocity.getY(),
                targetRotationalVelocity.getRadians());
        if (accelLimitedSpeeds.vxMetersPerSecond == -0) {
            accelLimitedSpeeds.vxMetersPerSecond = 0;
        }
        if (accelLimitedSpeeds.vyMetersPerSecond == -0) {
            accelLimitedSpeeds.vyMetersPerSecond = 0;
        }
        if (accelLimitedSpeeds.omegaRadiansPerSecond == -0) {
            accelLimitedSpeeds.omegaRadiansPerSecond = 0;
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(accelLimitedSpeeds);
        double minCos = 1;
        for(int i = 0; i < modules.length; i ++) {
            double cos = Math.cos((modules[i].getAngle().minus(states[i].angle)).getRadians());
            minCos = Math.min(minCos, Math.abs(cos));
        }
        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(modules[i].getAngle());
            states[i].speedMetersPerSecond *= minCos;
            modules[i].setState(states[i]);
        }
    }

    public void resetPose(Pose2d pose) {
        if (pose != null) {
            estimator.resetPose(pose);
        }
    }

    private void log() {
        Logger.recordOutput(getOutputLogPath("EstimatedPosition"), getPosition());
        Logger.recordOutput(getOutputLogPath("Heading"), gyro.getInputs().yaw);
        Logger.recordOutput(getOutputLogPath("TargetVelocity"), getCurrentSpeed());

        field.setRobotPose(getPosition());

    }

    public static class SwerveConstraints {
        private final double maxAngularAccel;
        private final double maxAngularVelocity;
        private final double maxLinearAccel;
        private final double maxLinearVelocity;

        public SwerveConstraints(double swerveMaxAngularVelocity, double swerveMaxAngularAccel,
                double swerveMaxLinerVelocity, double swerveMaxLinearAccel) {
            this.maxAngularAccel = swerveMaxAngularAccel;
            this.maxAngularVelocity = swerveMaxAngularVelocity;
            this.maxLinearAccel = swerveMaxLinearAccel;
            this.maxLinearVelocity = swerveMaxLinerVelocity;

        }

        public double getMaxAngularAccel() {
            return maxAngularAccel;
        }

        public double getMaxAngularVelocity() {
            return maxAngularVelocity;
        }

        public double getMaxLinerAccel() {
            return maxLinearAccel;
        }

        public double getMaxLinearVelocity() {
            return maxLinearVelocity;
        }

    }
}