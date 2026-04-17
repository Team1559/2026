package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
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

import java.util.PrimitiveIterator;
import java.util.function.Supplier;

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
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.pathplanner.lib.util.FlippingUtil;

import org.opencv.core.Mat.Tuple2;

import frc.lib.component.AngleComponent;
import frc.lib.component.AngleSensor;
import frc.lib.component.AngularVelocityComponent;
import frc.lib.intermediate.ChineseRemainderAngle;
import frc.lib.io.CanCoderIoBase;
import frc.lib.io.CanCoderIoReal;
import frc.lib.io.SparkFlexIoBase;
import frc.lib.io.SparkFlexIoReal;
import frc.lib.logging.LoggableSubsystem;
import frc.lib.util.ForwardReverseNeutral;

public class Shooter2026 extends LoggableSubsystem {
    private final Supplier<Pose2d> robotPositionSupplier;
    private final Supplier<ChassisSpeeds> robotSpeedSupplier;

    private AngularVelocity targetFlywheelVelocity = RPM.zero();
    // ^ Init at zero to fix crash due to null in isFlywheelReady
    private Translation3d targetFieldSpace;
    private boolean isShooting;
    private double timestampFlywheelNotReady;

    private ForwardReverseNeutral feedwheelState = ForwardReverseNeutral.NEUTRAL;
    private ForwardReverseNeutral flywheelState = ForwardReverseNeutral.NEUTRAL;

    private static final Rotation2d flapperAngle = Rotation2d.fromDegrees(60);

    private final AngleComponent turret;
    private final AngleSensor turretAngleSensor;
    private final SparkFlexIoBase flywheel; // AngularVelocityComponant
    private final AngularVelocityComponent feedWheel;

    private final Pose3d turretOffset;

    public static final Translation3d BLUE_HUB_LOCATION = new Translation3d(4.620, 4.030, Units.feetToMeters(6));
    public static final Translation3d RED_HUB_LOCATION = new Translation3d(
            FlippingUtil.flipFieldPosition(BLUE_HUB_LOCATION.toTranslation2d()))
            .plus(new Translation3d(0, 0, BLUE_HUB_LOCATION.getZ()));
    public static final Translation3d BLUE_FERRY_ONE = new Translation3d(Meters.of(2.525), Meters.of(6.558),
            Inches.of(0));
    public static final Translation3d BLUE_FERRY_TWO = new Translation3d(Meters.of(2.492), Meters.of(1.523),
            Inches.of(0));

    public static final LinearAcceleration GRAVITATIONAL_ACCEL = MetersPerSecondPerSecond.of(9.80665);
    private static final Time FLYWHEEL_DEBOUNCE = Seconds.of(0.15);
    private static final AngularVelocity TOLERANCE = RPM.of(20);
    private static final Time LOOKAHEAD = Seconds.of(1 / 20d);

    private static final Angle TURRET_MAX = Degrees.of(200);
    private static final Angle TURRET_MIN = Degrees.of(-60);
    private static final Angle TURRET_MID = TURRET_MAX.plus(TURRET_MIN).div(2);

    private static final int previousMeasurementCount = 5;
    CircularBuffer<PastAngle> previousAngles = new CircularBuffer<>(previousMeasurementCount);

    private record PastAngle(Double timestamp, Rotation2d angle) {
    }

    public Shooter2026(Supplier<Pose2d> robotPositionSupplier, Supplier<ChassisSpeeds> robotSpeedSupplier,
            Pose3d turretOffset, AngleComponent turret,
            SparkFlexIoBase flywheel, AngularVelocityComponent feedWheel,
            AngleSensor turretAngleSensor) {
        super("Shooter");
        this.robotPositionSupplier = robotPositionSupplier;
        this.robotSpeedSupplier = robotSpeedSupplier;
        this.turretOffset = turretOffset;
        this.turret = turret;
        this.flywheel = flywheel;
        this.feedWheel = feedWheel;
        this.turretAngleSensor = turretAngleSensor;
        this.addChild("Turret", turret);
        this.addChild("Flywheel", flywheel);
        this.addChild("FeedWheel", feedWheel);
        this.addChild("TurretAngleSensor", turretAngleSensor);
        useAbsoluteAngle();
    }

    public Shooter2026(Supplier<Pose2d> robotPositionSupplier, Supplier<ChassisSpeeds> robotSpeedSupplier) {
        this(robotPositionSupplier, robotSpeedSupplier,
                new Pose3d(Inches.of(5.5), Inches.of(5.5), Inches.of(28), Rotation3d.kZero),
                makeTurret(), makeFlywheel(), makeFeedwheel(),
                makeCrtAngleSensor());
    }

    private static SparkFlexIoBase makeFlywheel() {
        if (Logger.hasReplaySource()) {
            return new SparkFlexIoBase();
        } else {
            SparkFlexConfig config = new SparkFlexConfig();
            config.closedLoop.pid(0.0005, 0, 0.05);
            config.closedLoop.feedForward.kV(0.000151 * 12);
            config.inverted(false);
            config.idleMode(IdleMode.kCoast);
            config.voltageCompensation(12.0);
            return new SparkFlexIoReal(new SparkFlex(17, MotorType.kBrushless), config);
        }
    }

    private static AngularVelocityComponent makeFeedwheel() {
        AngularVelocityComponent sparkFlex;
        if (Logger.hasReplaySource()) {
            sparkFlex = new SparkFlexIoBase();
        } else {
            SparkFlexConfig config = new SparkFlexConfig();
            config.closedLoop.pid(0, 0, 0);
            config.closedLoop.feedForward.kV(0.00016 * 12);
            config.idleMode(IdleMode.kBrake);
            config.voltageCompensation(12.0);
            sparkFlex = new SparkFlexIoReal(new SparkFlex(16, MotorType.kBrushless), config);
        }
        return sparkFlex;
    }

    private static AngleComponent makeTurret() {
        SparkFlexIoBase sparkFlex;
        if (Logger.hasReplaySource()) {
            sparkFlex = new SparkFlexIoBase();
        } else {
            SparkFlexConfig config = new SparkFlexConfig();
            config.closedLoop.maxOutput(.15);
            config.closedLoop.minOutput(-.15);
            config.closedLoop.pid(0.8, 0.0005, 0);
            config.closedLoop.iZone(0.2);
            config.closedLoop.allowedClosedLoopError(Degrees.of(0.1 * 10).in(Rotations), ClosedLoopSlot.kSlot0); // 0.24
            config.voltageCompensation(12.0);
            config.inverted(true);
            config.idleMode(IdleMode.kBrake);
            sparkFlex = new SparkFlexIoReal(new SparkFlex(19, MotorType.kBrushless), config);
        }
        return sparkFlex.withRatio(10d).withLimits(TURRET_MIN, TURRET_MAX);
    }

    public Angle getAngle() {
        return turret.getAngle();
    }

    private static AngleSensor makeCrtAngleSensor() {

        CanCoderIoBase canCoderOne;
        CanCoderIoBase canCoderTwo;
        if (Logger.hasReplaySource()) {
            canCoderOne = new CanCoderIoBase();
            canCoderTwo = new CanCoderIoBase();
        } else {
            CANcoderConfiguration configOne = new CANcoderConfiguration();
            configOne.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            canCoderOne = new CanCoderIoReal(new CANcoder(14), configOne);
            CANcoderConfiguration configTwo = new CANcoderConfiguration();
            configTwo.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            canCoderTwo = new CanCoderIoReal(new CANcoder(15), configTwo);
        }

        return new ChineseRemainderAngle(21, 20, 200,
                canCoderOne.withOffset(Degrees.of(124.101563)),
                canCoderTwo.withOffset(Degrees.of(-13.095703)),
                Degrees.of(-240), Degrees.of(240));
    }

    public void setTargetFieldSpace(Translation3d target) {
        this.targetFieldSpace = target;
        previousAngles.clear();
    }

    public void setSpinFlywheel(boolean spinFlywheel) {
        if (spinFlywheel) {
            flywheelState = ForwardReverseNeutral.FORWARD;
        } else {
            flywheelState = ForwardReverseNeutral.NEUTRAL;
        }
    }

    public void setShooting(boolean shouldShoot) {
        this.isShooting = shouldShoot;
        if (isShooting) {
            feedwheelState = ForwardReverseNeutral.FORWARD;
        } else {
            feedwheelState = ForwardReverseNeutral.NEUTRAL;
        }
    }

    public void useAbsoluteAngle() {
        Angle crtAngle = turretAngleSensor.getAngle();
        turret.setPercievedAngle(crtAngle);
        logger().dashboard("CrtAngle", crtAngle);
    }

    public final void zeroTurret() {
        turret.setPercievedAngle(Degrees.of(0));
    }

    public final void ninteyTurret() {
        turret.setPercievedAngle(Degrees.of(90));
    }

    public AngularVelocity getTargetFlywheelVelocity() {
        return targetFlywheelVelocity;
    }

    public boolean isFlywheelReady() {

        if (Math.abs(flywheel.getCurrentVelocity().minus(targetFlywheelVelocity).in(RPM)) < TOLERANCE.in(RPM)
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

        return robotToTarget.minus(turretTranslation).rotateBy(turretRotation.unaryMinus());
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

    private static LinearVelocity calculateProjectileSpeedFixedAngle(Translation2d target, Rotation2d angle) {
        double projectileVelocity = Math
                .sqrt(GRAVITATIONAL_ACCEL.in(MetersPerSecondPerSecond) * (Math.pow(target.getX(), 2))
                        / (2 * Math.pow(angle.getCos(), 2) * (target.getX() * angle.getTan() - target.getY())));
        if (Double.isFinite(projectileVelocity)) {
            return MetersPerSecond.of(projectileVelocity);
        } else {
            return null;
        }

    }

    private static AngularVelocity calculateFlywheelVelocity(LinearVelocity projectileVelocity) {
        return RPM.of(1825 + -43.7 * projectileVelocity.in(FeetPerSecond)
                + 4.56 * Math.pow(projectileVelocity.in(FeetPerSecond), 2));

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

    private static Translation3d targetAtTimeT(Time t, Translation3d initialHubPose, Translation3d hubVelocity) {
        return initialHubPose.plus(hubVelocity.times(t.in(Seconds)));
    }

    private static Distance error(Time t, Translation3d initialHubPose, Translation3d hubVelocity, Rotation2d pitch) {
        Translation3d hubPoseAtT = targetAtTimeT(t, initialHubPose, hubVelocity);
        return hubPoseAtT.getMeasureZ().plus(GRAVITATIONAL_ACCEL.times(t).times(t).times(0.5))
                .minus(Meters.of(pitch.getTan() * hubPoseAtT.toTranslation2d().getNorm()));
    }

    private static LinearVelocity errorDerivative(Time t, Translation3d initialHubPose, Translation3d hubVelocity,
            Rotation2d pitch) {
        Translation3d hubPoseAtT = targetAtTimeT(t, initialHubPose, hubVelocity);
        LinearVelocity bigFreakingFraction = MetersPerSecond.of(pitch.getTan()
                * (((hubVelocity.getX() * hubPoseAtT.getX()) + (hubVelocity.getY() * hubPoseAtT.getY()))
                        / (hubPoseAtT.toTranslation2d().getNorm())));

        return hubVelocity.getMeasureZ().per(Second).plus(GRAVITATIONAL_ACCEL.times(t))
                .minus(bigFreakingFraction);
    }

    @Override
    public void periodic() {
        super.periodic();

        Pose2d robotPosition = robotPositionSupplier.get();
        Translation3d target = calculateTargetShooterSpace(targetFieldSpace, robotPosition, turretOffset);

        Angle chineseAngle = turretAngleSensor.getAngle();
        logger().debug("TargetFieldSpace", targetFieldSpace)
                .debug("TargetShooterSpace", target)
                .debug("FlywheelState", flywheelState)
                .debug("FeedwheelState", feedwheelState)
                .debug("OutputVelocity", flywheel.getCurrentVelocity())
                .debug("TurretAngle", turret.getAngle())
                .debug("IsFlywheelReady?", isFlywheelReady())
                .debug("CrtAngle", chineseAngle)
                .debug("ChineseDifference", chineseAngle.minus(turret.getAngle()));

        if (feedwheelState == ForwardReverseNeutral.FORWARD) {
            feedWheel.setVelocity(RPM.of(1500));
        } else if (feedwheelState == ForwardReverseNeutral.REVERSE) {
            feedWheel.setVelocity(RPM.of(-1000));
        } else {
            feedWheel.neutralOutput();
        }

        logger().dashboard("IsShooting", feedwheelState == ForwardReverseNeutral.FORWARD);
        if (target == null) {
            return;
        }
        Rotation2d angleToTarget = calculateAngleToTarget(target);
        Translation2d targetTurretSpace = calculateTargetLocationTurretSpace(target, angleToTarget);
        LinearVelocity projectileVelocity = calculateProjectileSpeedFixedAngle(targetTurretSpace, flapperAngle);
        if (projectileVelocity == null) {
            logger().debugPrintln("Projectile velocity is not finite, target is impossible to hit");
            return;
        }
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

        angleToTarget = calculateAngleToTarget(virtualTarget);
        Angle turretSetpoint = angleToTarget.getMeasure();
        while (turretSetpoint.lt(TURRET_MID.minus(Degrees.of(180)))) {
            turretSetpoint = turretSetpoint.plus(Rotations.one());
        }
        while (turretSetpoint.gt(TURRET_MID.plus(Degrees.of(180)))) {
            turretSetpoint = turretSetpoint.minus(Rotations.one());
        }
        Angle angleToTargetInRange = turretSetpoint;
        // if (previousAngles.size() > 0) {
        // PastAngle pastAngle = previousAngles.getLast();
        // AngularVelocity velocityOfTarget =
        // angleToTarget.minus(pastAngle.angle).getMeasure()
        // .div(Seconds.of(Timer.getTimestamp() - pastAngle.timestamp));
        // logger().debug("VelocityOfTarget", velocityOfTarget);
        // turretSetpoint = turretSetpoint.plus(velocityOfTarget.times(LOOKAHEAD));
        // }

        logger().debug("AngleToTarget", angleToTarget)
                .debug("TurretSetpoint", turretSetpoint);
        turret.setAngle(turretSetpoint);

        targetTurretSpace = calculateTargetLocationTurretSpace(virtualTarget, angleToTarget);
        logger().debug("VirtualTargetTurretSpace", targetTurretSpace);
        projectileVelocity = calculateProjectileSpeedFixedAngle(targetTurretSpace, flapperAngle);
        if (projectileVelocity == null) {
            logger().debugPrintln("Projectile velocity is not finite, target is impossible to hit");
            return;
        }
        logger().debug("ProjectileVelocitySetpoint", projectileVelocity);

        if (flywheelState == ForwardReverseNeutral.FORWARD
                && Double.isFinite(projectileVelocity.in(MetersPerSecond))) {
            targetFlywheelVelocity = calculateFlywheelVelocity(projectileVelocity);
            logger().debug("TargetFlywheelVelocity", targetFlywheelVelocity);
            flywheel.setVelocity(targetFlywheelVelocity);
        } else if (flywheelState == ForwardReverseNeutral.REVERSE) {
            flywheel.setVelocity(RPM.of(-1000));
        } else {
            logger().debug("TargetFlywheelVelocity", RPM.zero());
            flywheel.neutralOutput();
        }

        Angle turretError = turret.getAngle().minus(angleToTargetInRange);
        logger().dashboard("TurretOK", angleToTargetInRange.gt(TURRET_MIN) && angleToTargetInRange.lt(TURRET_MAX)
                && turretError.abs(Degrees) < 5).debug("TurretError", turretError);

        previousAngles.addFirst(new PastAngle(Timer.getTimestamp(), angleToTarget));
    }

    public void setTurretAngle(Angle setAngle) {
        turret.setAngle(setAngle);
        logger().debug("TargetTurretAngle", setAngle);
    }

    private static Rotation2d calculateAngleToTarget(Translation3d targetShooterSpace) {
        return new Rotation2d(targetShooterSpace.getX(), targetShooterSpace.getY());
    }

    private static Translation2d calculateTargetLocationTurretSpace(Translation3d target3d, Rotation2d turretAngle) {
        Translation3d targetOnPlane = target3d.rotateBy(new Rotation3d(turretAngle.unaryMinus()));
        if (Math.abs(targetOnPlane.getY()) >= 1E-6) {
            System.out.println("TargetOnPlane has a Y value of " + targetOnPlane.getY() + ", should be 0");
        }
        return new Translation2d(targetOnPlane.getX(), targetOnPlane.getZ()); // Z is up, is referred to as Y in 2d
    }

    public Command getAimCommand(Supplier<Translation3d> target) {
        return new InstantCommand(() -> setTargetFieldSpace(target.get()));
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
        flywheelState = ForwardReverseNeutral.NEUTRAL;
        feedwheelState = ForwardReverseNeutral.NEUTRAL;
    }

    public void reverseFeedwheel() {
        feedwheelState = ForwardReverseNeutral.REVERSE;
    }

    public void neutralFeedwheel() {
        feedwheelState = ForwardReverseNeutral.NEUTRAL;
    }

    public void reverseAll() {
        flywheelState = ForwardReverseNeutral.REVERSE;
        feedwheelState = ForwardReverseNeutral.REVERSE;
    }

}
