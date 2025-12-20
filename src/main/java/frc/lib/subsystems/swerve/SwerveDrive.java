package frc.lib.subsystems.swerve;

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
import frc.lib.components.gyro.GyroIo;
import frc.lib.subsystems.LoggableSubsystem;
import frc.lib.subsystems.swerve.SwerveModuleIo.SwerveInputs;
import frc.lib.subsystems.vision.VisionConsumer;

public class SwerveDrive extends LoggableSubsystem implements VisionConsumer {
    private final SwerveModuleIo[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator estimator;
    private final GyroIo gyro;

    private final Field2d field;

    private double maxLinearAcceleration = Double.POSITIVE_INFINITY;
    private Rotation2d maxRotationalAcceleration = Rotation2d.fromRadians(Double.POSITIVE_INFINITY);

    private ChassisSpeeds targetSpeed = new ChassisSpeeds();

    private final double ROBOT_PERIOD = 0.02;

    public SwerveDrive(String name, GyroIo gyro, SwerveModuleIo... modules) {
        super(name);
        this.gyro = gyro;
        this.modules = modules;
        Translation2d[] locations = new Translation2d[modules.length];
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < locations.length; i++) {
            locations[i] = modules[i].getLocation();
            SwerveInputs inputs = modules[i].getInputs();
            positions[i] = new SwerveModulePosition(inputs.distance, inputs.angle);
            addIo(modules[i], "Modules");
        }
        addIo(gyro, "Gyro");
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
                new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(5)), robotConfig,
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

    protected SwerveModuleIo[] getModules() {
        return modules.clone();
    }

    public ChassisSpeeds getCurrentSpeed() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < states.length; i++) {
            SwerveInputs inputs = modules[i].getInputs();
            states[i] = new SwerveModuleState(inputs.speed, inputs.angle);
        }
        return kinematics.toChassisSpeeds(states);
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < positions.length; i++) {
            SwerveInputs inputs = modules[i].getInputs();
            positions[i] = new SwerveModulePosition(inputs.distance, inputs.angle);
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

    private void updateOdometry() {
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
        Rotation2d targetRotationalAcceleration = targetRotationalVelocity.minus(currentRotationalVelocity) //TODO: debug when chasis is done
                .div(ROBOT_PERIOD);
        if (targetRotationalAcceleration.getRadians() > maxRotationalAcceleration.getRadians()) {
            if (targetRotationalAcceleration.getRadians() > 0) {
                targetRotationalAcceleration = maxRotationalAcceleration;
            } else {
                targetRotationalAcceleration = maxRotationalAcceleration.times(-1);
            }
        }
        targetRotationalVelocity = currentRotationalVelocity.plus(targetRotationalAcceleration.times(ROBOT_PERIOD));

        Logger.recordOutput(getLogPath("TargetLinearVelocity"), targetLinearVelocity);
        Logger.recordOutput(getLogPath("TargetRotationalVelocity"), targetRotationalVelocity);

        ChassisSpeeds accelLimitedSpeeds = new ChassisSpeeds(targetLinearVelocity.getX(), targetLinearVelocity.getY(),
                targetRotationalVelocity.getRadians());
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(accelLimitedSpeeds);
        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(modules[i].getInputs().angle);
            modules[i].setState(states[i]);
        }
    }

    public void resetPose(Pose2d pose) {
        if (pose != null) {
            estimator.resetPose(pose);
        }
    }

    private void log() {
        Logger.recordOutput(getLogPath("EstimatedPosition"), getPosition());
        Logger.recordOutput(getLogPath("Heading"), gyro.getInputs().yaw);
        Logger.recordOutput(getLogPath("TargetVelocity"), getCurrentSpeed());

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