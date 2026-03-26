package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.DirectionalThreeState;
import frc.lib.angular_position.AngularPositionComponent;
import frc.lib.angular_position.AngularPositionRatio;
import frc.lib.angular_position.AngularPositionSensor;
import frc.lib.angular_position.CanCoderIo;
import frc.lib.angular_position.ChineseRemainderAngle;
import frc.lib.angular_position.LimitedAngularPositionIntermediate;
import frc.lib.logging.LoggableSubsystem;
import frc.lib.velocity.AngularVelocityComponent;
import frc.lib.velocity.SparkFlexIo;
import frc.lib.velocity.SparkFlexReplayIo;

public class Shooter2026 extends LoggableSubsystem {
    private final Supplier<Pose2d> robotPositionSupplier;
    private final Supplier<ChassisSpeeds> robotSpeedSupplier;

    private AngularVelocity targetFlywheelVelocity = RPM.zero(); // Init at zero to fix crash due to null in
                                                                 // isFlywheelReady
    private Translation3d targetFieldSpace;
    private Translation2d barrierOffset;
    private boolean isShooting;
    private double timestampFlywheelNotReady;

    private DirectionalThreeState feedwheelState = DirectionalThreeState.NEUTRAL;
    private DirectionalThreeState flywheelState = DirectionalThreeState.NEUTRAL;

    private static final Rotation2d flapperAngle = Rotation2d.fromDegrees(59);

    private final AngularPositionComponent turret;
    private final AngularPositionSensor turretAngleSensor;
    private final AngularVelocityComponent flywheel;
    private final AngularPositionComponent flapper;
    private final AngularVelocityComponent feedWheel;

    private final Angle initialTurretOffset;

    private final Pose3d turretOffset;

    public static final Translation3d BLUE_HUB_LOCATION = new Translation3d(4.620, 4.030, Units.feetToMeters(6));
    public static final Translation3d RED_HUB_LOCATION = new Translation3d(
            FlippingUtil.flipFieldPosition(BLUE_HUB_LOCATION.toTranslation2d()))
            .plus(new Translation3d(0, 0, BLUE_HUB_LOCATION.getZ()));
    public static final Translation3d BLUE_FERRY_ONE = new Translation3d(Inches.of(27), Inches.of(317.69 - 48),
            Inches.of(0));
    public static final Translation3d BLUE_FERRY_TWO = new Translation3d(Inches.of(27), Inches.of(48), Inches.of(0));

    public static final LinearAcceleration GRAVITATIONAL_ACCEL = MetersPerSecondPerSecond.of(9.80665);
    private static final Time FLYWHEEL_DEBOUNCE = Seconds.of(0.15);
    private static final AngularVelocity tolerance = RPM.of(20);

    public Shooter2026(Supplier<Pose2d> robotPositionSupplier, Supplier<ChassisSpeeds> robotSpeedSupplier,
            Pose3d turretOffset, AngularPositionComponent turret,
            AngularPositionComponent flapper, AngularVelocityComponent flywheel, AngularVelocityComponent feedWheel,
            AngularPositionSensor turretAngleSensor) {
        super("Shooter");
        this.robotPositionSupplier = robotPositionSupplier;
        this.robotSpeedSupplier = robotSpeedSupplier;
        this.turretOffset = turretOffset;
        this.turret = turret;
        this.flywheel = flywheel;
        this.flapper = flapper;
        this.feedWheel = feedWheel;
        this.turretAngleSensor = turretAngleSensor;
        initialTurretOffset = turret.getAngle().minus(turretAngleSensor.getAngle());
        this.addChildren(turret, flywheel, flapper, feedWheel, turretAngleSensor);
        zeroTurret();
    }

    public Shooter2026(Supplier<Pose2d> robotPositionSupplier, Supplier<ChassisSpeeds> robotSpeedSupplier,
            boolean isReplay) {
        this(robotPositionSupplier, robotSpeedSupplier,
                new Pose3d(Inches.of(5.5), Inches.of(5.5), Inches.of(28), Rotation3d.kZero),
                makeTurret(isReplay), null, makeFlywheel(isReplay), makeFeedwheel(isReplay),
                makeCrtAngleSensor());
    }

    private static AngularVelocityComponent makeFlywheel(boolean isReplay) {
        if (isReplay){
            return new SparkFlexReplayIo("Flywheel");
        } else {
            SparkFlexConfig config = new SparkFlexConfig();
            config.closedLoop.pid(0.0005, 0, 0.05);
            config.closedLoop.feedForward.kV(0.000151); // Volts per rpm (0.000153)
            config.inverted(false);
            config.idleMode(IdleMode.kCoast);
            config.voltageCompensation(12.0);
            return new SparkFlexIo("Flywheel", new SparkFlex(17, MotorType.kBrushless), config);
        }
    }

    private static AngularVelocityComponent makeFeedwheel(boolean isReplay) {
        AngularVelocityComponent sparkFlex;
        if (isReplay){
            sparkFlex = new SparkFlexReplayIo("FeedWheelMotor");
        } else {
            SparkFlexConfig config = new SparkFlexConfig(); // TODO: Configure
            config.closedLoop.pid(0, 0, 0);
            config.closedLoop.feedForward.kV(0.00016); // Volts per rpm
            config.idleMode(IdleMode.kBrake);
            config.voltageCompensation(12.0);
            sparkFlex = new SparkFlexIo("Feedwheel", new SparkFlex(16, MotorType.kBrushless), config);
        }
        return sparkFlex;
    }

    private static AngularPositionComponent makeTurret(boolean isReplay) {
        SparkFlexReplayIo sparkFlex;
        if (isReplay) {
            sparkFlex = new SparkFlexReplayIo("TurretMotor");
        } else {
            SparkFlexConfig config = new SparkFlexConfig(); // TODO: Configure
            config.closedLoop.maxOutput(.15);
            config.closedLoop.minOutput(-.15);
            config.closedLoop.pid(.8, 0.0005, 0); // (0.1, 0.0008, 0.01);
            config.closedLoop.iZone(0.2);
            config.closedLoop.allowedClosedLoopError(Degrees.of(.24 * 10).in(Rotations), ClosedLoopSlot.kSlot0);
            config.voltageCompensation(12.0);
            config.inverted(true);
            config.idleMode(IdleMode.kBrake);
            sparkFlex = new SparkFlexIo("TurretMotor", new SparkFlex(19, MotorType.kBrushless), config);
        }
        return new LimitedAngularPositionIntermediate("Turret", Degrees.of(-90), Degrees.of(150),
                new AngularPositionRatio("GearRatio", 10d, sparkFlex));
    }

    public Angle getAngle() {
        return turret.getAngle();
    }

    private static SparkFlexIo makeFlapper() {
        SparkFlexConfig config = new SparkFlexConfig(); // TODO: Configure
        config.closedLoop.pid(0, 0, 0);
        config.voltageCompensation(12.0);
        return new SparkFlexIo("Flapper", new SparkFlex(0, MotorType.kBrushless), config); // TODO: ID motor
    }

    private static AngularPositionSensor makeCrtAngleSensor() {
        CANcoderConfiguration configOne = new CANcoderConfiguration();
        configOne.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CanCoderIo canCoderOne = new CanCoderIo("TurretGearOne", new CANcoder(14), Degrees.of(125.332031), configOne);
        CANcoderConfiguration configTwo = new CANcoderConfiguration();
        configTwo.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        CanCoderIo canCoderTwo = new CanCoderIo("TurretGearTwo", new CANcoder(15), Degrees.of(-4.746094), configTwo);

        return new ChineseRemainderAngle("CrtAngleSensor", 21, 19, 200, canCoderOne, canCoderTwo, Degrees.of(-180),
                Degrees.of(180));
    }

    public void setTargetFieldSpace(Translation3d target, Translation2d barrierOffset) {
        this.targetFieldSpace = target;
        this.barrierOffset = barrierOffset;
    }

    public void setSpinFlywheel(boolean spinFlywheel) {
        if (spinFlywheel) {
            flywheelState = DirectionalThreeState.FOWARD;
        } else {
            flywheelState = DirectionalThreeState.NEUTRAL;
        }
    }

    public void setShooting(boolean shouldShoot) {
        this.isShooting = shouldShoot;
        if (isShooting) {
            feedwheelState = DirectionalThreeState.FOWARD;
        } else {
            feedwheelState = DirectionalThreeState.NEUTRAL;
        }
    }

    public void useAbsoluteAngle() {
        Angle crtAngle = turretAngleSensor.getAngle();
        turret.setPercievedAngle(crtAngle);
        logger().dashboard("CrtAngle", crtAngle);
    }

    public void zeroTurret() {
        turret.setPercievedAngle(Degrees.of(0));
    }

    public AngularVelocity getTargetFlywheelVelocity() {
        return targetFlywheelVelocity;
    }

    public boolean isFlywheelReady() {

        if (Math.abs(flywheel.getCurrentVelocity().minus(targetFlywheelVelocity).in(RPM)) < tolerance.in(RPM)
                && targetFlywheelVelocity.gt(RPM.zero())) {
            return Timer.getTimestamp() - timestampFlywheelNotReady > FLYWHEEL_DEBOUNCE.in(Seconds);
        } else {
            timestampFlywheelNotReady = Timer.getTimestamp();
            return false;
        }
    }

    private static Translation3d calculateTargetShooterSpace(Translation3d targetFieldSpace, Pose2d robotPosition,
            Pose3d turretOffset) {
        if (targetFieldSpace == null) {
            return null;
        }
        // Target Position in robot coordinates
        Translation3d robotTranslation = new Translation3d(robotPosition.getTranslation());
        Rotation3d robotRotation = new Rotation3d(robotPosition.getRotation());
        Translation3d robotToTarget = targetFieldSpace.minus(robotTranslation).rotateBy(robotRotation.unaryMinus());

        // Target position in turret coordinates
        Translation3d turretTranslation = turretOffset.getTranslation();
        Rotation3d turretRotation = turretOffset.getRotation();
        Translation3d turretToTarget = robotToTarget.minus(turretTranslation).rotateBy(turretRotation.unaryMinus());

        return turretToTarget;
    }

    private static Translation3d shooterSpaceToFieldSpace(Translation3d translationShooterSpace, Pose2d robotPosition,
            Pose3d turretOffset) {
        // Location in robot space
        Translation3d robotSpace = translationShooterSpace.rotateBy(turretOffset.getRotation())
                .plus(turretOffset.getTranslation());
        // Location in field space
        return robotSpace.rotateBy(new Rotation3d(robotPosition.getRotation()))
                .plus(new Translation3d(robotPosition.getTranslation()));
    }

    // private void aimVariableAngle(Translation2d target) {
    // Translation2d barrierPoint = target.minus(barrierOffset);

    // // Quadratic from 3 points: y - y_1 = (x - x_1) / (x_3 - x_2) * ( ((y_3 -
    // y_1) *
    // // (x - x_2)) / (x_3 - x_1) - (y_2 - y_1)(x - x_3) / (x_2 - x_1) )
    // // One of the points = 0, 0: y = x / (x_3 - x_2) * ( y_3 * (x - x_2 ) / x_3 -
    // // y_2 * (x - x_3) / x_2 )
    // // Derivative of previous: d_y / d_x = 1 / (x_3 - x_2) * ( ((2y_3 - y_3 *
    // x_2) /
    // // x_3) - ((2y_2 * x - y_2 * x_3) / (x_2) )
    // // X coordinate of vertex: x = ((y_3 * x_2 ^2) - (y_2 * x_3 ^2)) / ((2x_2 *
    // y_3)
    // // - (2x_3 * y_2))

    // double x2 = target.getX();
    // double y2 = target.getY();
    // double x3 = barrierPoint.getX();
    // double y3 = barrierPoint.getY();

    // double vertexX = ((y3 * x2 * x2) - (y2 * x3 * x3)) / ((2 * x2 * y3) - (2 * x3
    // * y2));
    // double vertexY = vertexX / (x3 - x2) * (y3 * (vertexX - x2) / x3 - y2 *
    // (vertexX - x3) / x2);

    // double velocityY = Math.sqrt(2 *
    // GRAVITATIONAL_ACCEL.in(MetersPerSecondPerSecond) * vertexY);
    // double timeToApex = velocityY /
    // GRAVITATIONAL_ACCEL.in(MetersPerSecondPerSecond);

    // double velocityX = vertexX / timeToApex;

    // Translation2d projectileVelocity = new Translation2d(velocityX, velocityY);
    // logger().debug("TargetProjectileVelocity",
    // projectileVelocity);

    // flapper.setAngle(projectileVelocity.getAngle().getMeasure());

    // if (spinFlywheel) {
    // AngularVelocity targetAngularVelocity = RPM.of(projectileVelocity.getNorm());
    // // TODO: create actual
    // // equation
    // logger().debug("TargetFlywheelVelocity",
    // targetAngularVelocity);
    // flywheel.setVelocity(targetAngularVelocity);
    // } else {
    // flywheel.neutralOutput();
    // logger().debug("TargetFlywheelVelocity", RPM.zero());
    // }
    // }

    private static LinearVelocity calculateProjectileSpeedFixedAngle(Translation2d target, Rotation2d angle) {
        return MetersPerSecond
                .of(Math.sqrt(GRAVITATIONAL_ACCEL.in(MetersPerSecondPerSecond) * (Math.pow(target.getX(), 2))
                        / (2 * Math.pow(angle.getCos(), 2) * (target.getX() * angle.getTan() - target.getY()))));

    }

    private static AngularVelocity calculateFlywheelVelocity(LinearVelocity projectileVelocity) {
        return RPM.of(1100 + 21.4 * projectileVelocity.in(FeetPerSecond)
                + 3.16 * Math.pow(projectileVelocity.in(FeetPerSecond), 2));
    }

    private static Translation3d calculateVirtualTarget(Time initialGuess, Translation3d initialHubPose,
            Translation3d hubVelocity, Rotation2d pitch) {
        Time guess = initialGuess;
        for (int i = 0; i < 10; i++) {
            Distance error = error(guess, initialHubPose, hubVelocity, pitch);
            if (error.lt(Meters.of(0.01))) {
                break;
            }
            LinearVelocity errorDerivative = errorDerivative(guess, initialHubPose, hubVelocity, pitch);
            guess = guess.minus(error.div(errorDerivative));
        }

        return targetAtTimeT(guess, initialHubPose, hubVelocity);

    }

    private static Translation3d targetAtTimeT(Time T, Translation3d initialHubPose, Translation3d hubVelocity) {
        return initialHubPose.plus(hubVelocity.times(T.in(Seconds)));
    }

    private static Distance error(Time T, Translation3d initialHubPose, Translation3d hubVelocity, Rotation2d pitch) {
        Translation3d hubPoseAtT = targetAtTimeT(T, initialHubPose, hubVelocity);
        return hubPoseAtT.getMeasureZ().plus(GRAVITATIONAL_ACCEL.times(T).times(T).times(0.5))
                .minus(Meters.of(pitch.getTan() * hubPoseAtT.toTranslation2d().getNorm()));
    }

    private static LinearVelocity errorDerivative(Time T, Translation3d initialHubPose, Translation3d hubVelocity,
            Rotation2d pitch) {
        Translation3d hubPoseAtT = targetAtTimeT(T, initialHubPose, hubVelocity);
        LinearVelocity bigFreakingFraction = MetersPerSecond.of(pitch.getTan()
                * (((hubVelocity.getX() * hubPoseAtT.getX()) + (hubVelocity.getY() * hubPoseAtT.getY()))
                        / (hubPoseAtT.toTranslation2d().getNorm())));

        return hubVelocity.getMeasureZ().per(Second).plus(GRAVITATIONAL_ACCEL.times(T))
                .minus(bigFreakingFraction);
    }

    @Override
    public void periodic() {
        super.periodic();
        Pose2d robotPosition = robotPositionSupplier.get();
        Translation3d target = calculateTargetShooterSpace(targetFieldSpace, robotPosition, turretOffset);
        if (target != null) {
            Rotation2d turretAngle = calculateTargetTurretAngle(target);
            Translation2d targetTurretSpace = calculateTargetLocationTurretSpace(target, turretAngle);
            LinearVelocity projectileVelocity = calculateProjectileSpeedFixedAngle(targetTurretSpace, flapperAngle);

            Time initialGuess = targetTurretSpace.getMeasureX().div(projectileVelocity.times(flapperAngle.getCos()));
            logger().debug("InitialGuess", initialGuess);

            ChassisSpeeds robotSpeed = robotSpeedSupplier.get();
            Translation3d targetSpeed = new Translation3d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond, 0)
                    .unaryMinus().rotateBy(turretOffset.getRotation().unaryMinus());
            logger().debug("TargetSpeed", targetSpeed);

            Translation3d virtualTarget = calculateVirtualTarget(initialGuess, target, targetSpeed, flapperAngle);
            logger().debug("VirtualTarget", virtualTarget)
                    .debug("VirtualTargetFieldSpace",
                            shooterSpaceToFieldSpace(virtualTarget, robotPosition, turretOffset));

            turretAngle = calculateTargetTurretAngle(virtualTarget);
            logger().debug("TurretAngleSetpoint", turretAngle);
            turret.setAngle(turretAngle.getMeasure());

            targetTurretSpace = calculateTargetLocationTurretSpace(virtualTarget, turretAngle);
            logger().debug("VirtualTargetTurretSpace", targetTurretSpace);
            projectileVelocity = calculateProjectileSpeedFixedAngle(targetTurretSpace, flapperAngle);
            logger().debug("ProjectileVelocitySetpoint", projectileVelocity);

            if (flywheelState == DirectionalThreeState.FOWARD
                    && Double.isFinite(projectileVelocity.in(MetersPerSecond))) {
                targetFlywheelVelocity = calculateFlywheelVelocity(projectileVelocity);
                logger().debug("TargetFlywheelVelocity", targetFlywheelVelocity);
                flywheel.setVelocity(targetFlywheelVelocity);
            } else if (flywheelState == DirectionalThreeState.REVERSE) {
                flywheel.setVelocity(RPM.of(-1000));
            } else {
                logger().debug("TargetFlywheelVelocity", RPM.zero());
                flywheel.neutralOutput();
            }

            Angle turretError = turret.getAngle().minus(turretAngle.getMeasure());
            logger().dashboard("TurretOK",
                    turretAngle.getDegrees() > -90 && turretAngle.getDegrees() < 150 && turretError.abs(Degrees) < 5) // True
                                                                                                                      // if
                                                                                                                      // turret
                                                                                                                      // is
                                                                                                                      // in
                                                                                                                      // range
                                                                                                                      // of
                                                                                                                      // target,
                                                                                                                      // RIT
                    .debug("TurretError", turretError);
        }

        logger().debug("TargetFieldSpace", targetFieldSpace)
                .debug("TargetShooterSpace", target)
                .debug("FlywheelStaet", flywheelState)
                .debug("FeedwheelState", feedwheelState)
                .debug("OutputVelocity", flywheel.getCurrentVelocity())
                .debug("TurretAngle", turret.getAngle())
                .debug("IsFlywheelReady?", isFlywheelReady());
        if (DriverStation.isTest() || DriverStation.isDisabled()) {
            logger().dashboard("CrtAngle", turretAngleSensor.getAngle());
        }

        if (feedwheelState == DirectionalThreeState.FOWARD) {
            feedWheel.setVelocity(RPM.of(1500));
        } else if (feedwheelState == DirectionalThreeState.REVERSE) {
            feedWheel.setVelocity(RPM.of(-1000));
        } else {
            feedWheel.neutralOutput();
        }

        logger().dashboard("IsShooting", feedwheelState == DirectionalThreeState.FOWARD);
    }

    public void setTurretAngle(Angle setAngle) {
        turret.setAngle(setAngle);
        logger().debug("TargetTurretAngle", setAngle);
    }

    private static Rotation2d calculateTargetTurretAngle(Translation3d targetShooterSpace) {
        return new Rotation2d(targetShooterSpace.getX(), targetShooterSpace.getY());
    }

    private static Translation2d calculateTargetLocationTurretSpace(Translation3d target3d, Rotation2d turretAngle) {
        Translation3d targetOnPlane = target3d.rotateBy(new Rotation3d(turretAngle.unaryMinus()));
        if (!(Math.abs(targetOnPlane.getY()) < 1E-6)) {
            System.out.println("TargetOnPlane has a Y value of " + targetOnPlane.getY() + ", should be 0");
        }
        return new Translation2d(targetOnPlane.getX(), targetOnPlane.getZ()); // Z is up, is referred to as Y in 2d
    }

    public Command getAimCommand(Supplier<Translation3d> target) {
        return new InstantCommand(() -> setTargetFieldSpace(target.get(), null));
    }

    public static Translation3d ourHubLocation() {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? RED_HUB_LOCATION : BLUE_HUB_LOCATION;
    }

    public Translation3d targetLocation() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Translation3d target;
        Pose2d robotLocation = robotPositionSupplier.get();
        if (alliance == Alliance.Red) {
            robotLocation = FlippingUtil.flipFieldPose(robotLocation);
        }
        if (robotLocation.getMeasureX().lt(Inches.of(182.11))) {
            target = BLUE_HUB_LOCATION;
        } else if (robotLocation.getMeasureY().gt(Inches.of(158.84))) {
            target = BLUE_FERRY_ONE;
        } else {
            target = BLUE_FERRY_TWO;
        }

        if (alliance == Alliance.Red) {
            return flip(target);
        }
        return target;
    }

    public static Translation3d flip(Translation3d t) {
        Translation2d flipped2d = FlippingUtil.flipFieldPosition(t.toTranslation2d());
        return new Translation3d(flipped2d.getX(), flipped2d.getY(), t.getZ());
    }

    public boolean isShooting() {
        return isShooting;
    }

    public void neutralAll() {
        flywheelState = DirectionalThreeState.NEUTRAL;
        feedwheelState = DirectionalThreeState.NEUTRAL;
    }

    public void reverseFeedwheel() {
        feedwheelState = DirectionalThreeState.REVERSE;
    }

    public void neutralFeedwheel() {
        feedwheelState = DirectionalThreeState.NEUTRAL;
    }

    public void reverseAll() {
        flywheelState = DirectionalThreeState.REVERSE;
        feedwheelState = DirectionalThreeState.REVERSE;
    }

}
