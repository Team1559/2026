package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.FlippingUtil;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.LoggableSubsystem;
import frc.lib.angularPosition.AngularPositionComponent;
import frc.lib.angularPosition.AngularPositionRatio;
import frc.lib.angularPosition.AngularPositionSensor;
import frc.lib.angularPosition.CanCoderIo;
import frc.lib.angularPosition.ChineseBaby;
import frc.lib.angularPosition.LimitedAngularPositionIntermediate;
import frc.lib.velocity.AngularVelocityComponent;
import frc.lib.velocity.SparkFlexIo;

public class Shooter2026 extends LoggableSubsystem {
    private final Supplier<Pose2d> robotPositionSupplier;
    
    private AngularVelocity targetFlywheelVelocity;
    private Translation3d targetFieldSpace;
    private Translation2d barrierOffset;
    private boolean spinFlywheel;
    private boolean spinFeedwheel;
    private double timestampFlywheelNotReady;
    private final Rotation2d flapperAngle = Rotation2d.fromDegrees(57);
    
    private final AngularPositionComponent turret; //APC
    private final AngularPositionSensor turretAngleSensor;
    private final AngularVelocityComponent flywheel; //Velocity Component
    private final AngularPositionComponent flapper; //Angular Position Component
    private final AngularVelocityComponent feedWheel; //VelocityComponent

    private final Angle initialTurretOffset;
    
    private final Pose3d turretOffset;

    public final Translation3d blueHubLocation = new Translation3d(4.620, 4.030, Units.feetToMeters(6.0));
    public final Translation3d redHubLocation = new Translation3d(FlippingUtil.flipFieldPosition(blueHubLocation.toTranslation2d())).plus(new Translation3d(0, 0, blueHubLocation.getZ()));
    
    //TO​DO: calibrate based on gravity at field/make constants class for venue gravitational accelerations
    public static final LinearAcceleration g = MetersPerSecondPerSecond.of(9.80665); //Units: N/kg

    public static void main(String[] args) {
        Translation2d target = new Translation2d(5, 3.5);
        Rotation2d angle = Rotation2d.fromDegrees(57);
        //LinearVelocity velocty = FeetPerSecond.of(Math.sqrt(g * (Math.pow(target.getX(), 2) / (2 * Math.pow(angle.getCos(), 2) * (target.getX() * angle.getTan()) - target.getY()))));
        // LinearVelocity veloctyIsaac = FeetPerSecond.of(Math.sqrt((g * Math.pow(target.getX(), 2)) / ((2 * Math.pow(angle.getCos(), 2) * (target.getX() * angle.getTan())) - target.getY())));


        // LinearVelocity KYLE = FeetPerSecond.of(Math.sqrt(g * ((Math.pow(target.getX(), 2)) / (2 * Math.pow(angle.getCos(), 2) * ((target.getX() * angle.getTan()) - target.getY())))));


        LinearVelocity velocity = FeetPerSecond.of(Math.sqrt(g.in(FeetPerSecondPerSecond) * (Math.pow(target.getX(), 2)) / (2 * Math.pow(angle.getCos(), 2) * (target.getX() * angle.getTan() - target.getY()))));
        System.out.println(velocity);
    }
    
    public Shooter2026(Supplier<Pose2d> robotPositionSupplier, Pose3d turretOffset,  AngularPositionComponent turret, AngularPositionComponent flapper, AngularVelocityComponent flywheel, AngularVelocityComponent feedWheel, AngularPositionSensor turretAngleSensor) {
        super("Shooter");
        this.robotPositionSupplier = robotPositionSupplier;
        this.turretOffset = turretOffset;
        this.turret = turret;
        this.flywheel = flywheel;
        this.flapper = flapper;
        this.feedWheel = feedWheel;
        this.turretAngleSensor = turretAngleSensor;
        initialTurretOffset = turret.getAngle().minus(turretAngleSensor.getAngle());
        this.addChildren(turret, flywheel, flapper, feedWheel, turretAngleSensor);
        turret.setPercievedAngle(turretAngleSensor.getAngle());
    }

    public Shooter2026(Supplier<Pose2d> robotPositionSupplier){
        this(robotPositionSupplier, new Pose3d(Units.inchesToMeters(9), Units.inchesToMeters(7), Units.inchesToMeters(30), Rotation3d.kZero), makeTurret(), null, makeFlywheel(), makeFeedwheel(), makeBaby());//TODO: Offset
    }

    private static SparkFlexIo makeFlywheel(){
        SparkFlexConfig config = new SparkFlexConfig(); //TODO: Configure
        config.closedLoop.pid(0, 0, 0);//1/20000d
        config.closedLoop.feedForward.kV(0.000153);    //Volts per rpm
        config.inverted(true);
        config.idleMode(IdleMode.kCoast);
        config.voltageCompensation(12.0);
        return new SparkFlexIo("Flywheel", new SparkFlex(17, MotorType.kBrushless), config);
    }
    
    private static SparkFlexIo makeFeedwheel(){
        SparkFlexConfig config = new SparkFlexConfig(); //TODO: Configure
        config.closedLoop.pid(0, 0, 0);
        config.closedLoop.feedForward.kV(1 / 565.0);    //Volts per rpm
        config.idleMode(IdleMode.kBrake);
        config.voltageCompensation(12.0);
        return new SparkFlexIo("Feedwheel", new SparkFlex(16, MotorType.kBrushless), config);
    }

    private static AngularPositionComponent makeTurret() {
        SparkFlexConfig config = new SparkFlexConfig(); //TODO: Configure
        config.closedLoop.pid(0.1, 0.000001, 0); //P: 2, i:0, d:0.2
        config.voltageCompensation(12.0);
        config.inverted(true);
        return new LimitedAngularPositionIntermediate("Turret", Degrees.of(-100), Degrees.of(100), new AngularPositionRatio("GearRatio", 30, new SparkFlexIo("TurretMotor", new SparkFlex(19, MotorType.kBrushless), config)));
    }

    private static SparkFlexIo makeFlapper() {
        SparkFlexConfig config = new SparkFlexConfig(); //TODO: Configure
        config.closedLoop.pid(0, 0, 0);
        config.voltageCompensation(12.0);
        return new SparkFlexIo("Flapper", new SparkFlex(0, MotorType.kBrushless), config); //TODO: ID motor
    }

    private static AngularPositionSensor makeBaby(){
        CANcoderConfiguration configOne = new CANcoderConfiguration();
        configOne.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        CanCoderIo canCoderOne = new CanCoderIo("TurretGearOne", new CANcoder(14), Radians.of(1.756408), configOne); //TODO: calibrate offsets     2.006447

        CANcoderConfiguration configTwo = new CANcoderConfiguration();
        configTwo.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CanCoderIo canCoderTwo = new CanCoderIo("TurretGearTwo", new CANcoder(15), Radians.of(-2.357728), configTwo); //TODO: calibrate offsets     -1.79169

        return new ChineseBaby("ChineseBaby", 19, 21, 200, canCoderTwo, canCoderOne, Degrees.of(-180), Degrees.of(180));
    }

    public void setTargetFieldSpace(Translation3d target, Translation2d barrierOffset) {
        this.targetFieldSpace = target;
        this.barrierOffset = barrierOffset;
    }
    
    public void setSpinFlywheel(boolean spinFlywheel) {
        this.spinFlywheel = spinFlywheel;
    }

    public void setSpinFeedwheel(boolean spinFeedwheel) {
        this.spinFeedwheel = spinFeedwheel;
    }
    
    public AngularVelocity getTargetFlywheelVelocity() {
        return targetFlywheelVelocity;
    }

    public boolean isFlywheelReady() {
        double debounceSeconds = 0.15;
        AngularVelocity tolerance = RPM.of(50);
        if (Math.abs(flywheel.getCurrentVelocity().minus(targetFlywheelVelocity).in(RPM)) < tolerance.in(RPM)) {
            return Timer.getTimestamp() - timestampFlywheelNotReady > debounceSeconds;
        } else {
            timestampFlywheelNotReady = Timer.getTimestamp();
            return false;
        }
    }

    private Translation3d calculateTargetShooterSpace() {
        if (targetFieldSpace == null) {
            return null;
        }
        //Target Position in robot coordinates
        Pose2d robotPosition = robotPositionSupplier.get();
        Translation3d robotTranslation = new Translation3d(robotPosition.getTranslation());
        Rotation3d robotRotation = new Rotation3d(robotPosition.getRotation());
        Translation3d robotToTarget = targetFieldSpace.minus(robotTranslation).rotateBy(robotRotation.unaryMinus());
        
        //Target position in turret coordinates
        Translation3d turretTranslation = turretOffset.getTranslation();
        Rotation3d turretRotation = turretOffset.getRotation();
        Translation3d turretToTarget = robotToTarget.minus(turretTranslation).rotateBy(turretRotation.unaryMinus());
        
        return turretToTarget;
    }

    private void aimDynamic(Translation2d target) {
        Translation2d barrierPoint = target.minus(barrierOffset);
        
        //Quadratic from 3 points: y - y_1 = (x - x_1) / (x_3 - x_2) * ( ((y_3 - y_1) * (x - x_2)) / (x_3 - x_1) - (y_2 - y_1)(x - x_3) / (x_2 - x_1) )
        //One of the points = 0, 0: y = x / (x_3 - x_2) * ( y_3 * (x - x_2 ) / x_3 - y_2 * (x - x_3) / x_2 )
        //Derivative of previous: d_y / d_x = 1 / (x_3 - x_2) * ( ((2y_3 - y_3 * x_2) / x_3) - ((2y_2 * x - y_2 * x_3) / (x_2) )
        //X coordinate of vertex: x = ((y_3 * x_2 ^2) - (y_2 * x_3 ^2)) / ((2x_2 * y_3) - (2x_3 * y_2))
        
        double x2 = target.getX();
        double y2 = target.getY();
        double x3 = barrierPoint.getX();
        double y3 = barrierPoint.getY();

        double vertexX = ((y3 * x2 * x2) - (y2 * x3 * x3)) / ((2 * x2 * y3) - (2 * x3 * y2));
        double vertexY = vertexX / (x3 - x2) * ( y3 * (vertexX - x2 ) / x3 - y2 * (vertexX - x3) / x2 );

        double velocityY = Math.sqrt(2 * g.in(MetersPerSecondPerSecond) * vertexY);
        double timeToApex = velocityY / g.in(MetersPerSecondPerSecond);

        double velocityX = vertexX / timeToApex;

        Translation2d projectileVelocity = new Translation2d(velocityX, velocityY);
        Logger.recordOutput(getOutputLogPath("TargetProjectileVelocity"), projectileVelocity);

        flapper.setTargetAngle(projectileVelocity.getAngle().getMeasure());

        if (spinFlywheel){
            AngularVelocity targetAngularVelocity = RPM.of(projectileVelocity.getNorm());    //TODO: create actual equation 
            Logger.recordOutput(getOutputLogPath("TargetFlywheelVelocity"), targetAngularVelocity);
            flywheel.setTargetVelocity(targetAngularVelocity);
        } else {
            flywheel.stop();
            Logger.recordOutput(getOutputLogPath("TargetFlywheelVelocity"), RPM.of(0));
        }
    }

    private void aimFixed(Translation2d target, Rotation2d angle) {
        LinearVelocity projectileVelocity = MetersPerSecond.of(Math.sqrt(g.in(MetersPerSecondPerSecond) * (Math.pow(target.getX(), 2)) / (2 * Math.pow(angle.getCos(), 2) * (target.getX() * angle.getTan() - target.getY()))));
        Logger.recordOutput(getOutputLogPath("TargetProjectileVelocity"), projectileVelocity);

        if(spinFlywheel && Double.isFinite(projectileVelocity.in(MetersPerSecond))){
            targetFlywheelVelocity = RPM.of(0);    //TODO: create actual equation 
            Logger.recordOutput(getOutputLogPath("TargetFlywheelVelocity"), targetFlywheelVelocity);
            // flywheel.setTargetVelocity(targetAngularVelocity);
        } else {
            flywheel.stop();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        Translation3d target = calculateTargetShooterSpace();
        if (target != null) {
            Translation2d targetTurretSpace = aimTurret(target);
            Logger.recordOutput(getOutputLogPath("TargetTurretSpace"), targetTurretSpace);
            aimFixed(targetTurretSpace, flapperAngle);
        }
        Logger.recordOutput(getOutputLogPath("TargetFieldSpace"), targetFieldSpace);
        Logger.recordOutput(getOutputLogPath("TargetShooterSpace"), target);
        Logger.recordOutput(getOutputLogPath("SpinningFlywheel"), spinFlywheel);
        Logger.recordOutput(getOutputLogPath("ActualTurretAngle"), turretAngleSensor.getAngle());
        Logger.recordOutput(getOutputLogPath("OutputVelocity"), flywheel.getCurrentVelocity());
        Logger.recordOutput(getOutputLogPath("TurretAngle"), turret.getAngle());
        Logger.recordOutput(getOutputLogPath("IsFlywheelReady?"), isFlywheelReady());


        if(spinFeedwheel) {
            feedWheel.setTargetVelocity(RPM.of(500));
        } else {
            feedWheel.stop();
        }
        if(spinFlywheel) {
            flywheel.setTargetVelocity(RPM.of(5000));
        } else {
            flywheel.stop();
        }
    }

    public void setTurretAngle(Angle setAngle){
        turret.setTargetAngle(setAngle);
        Logger.recordOutput(getOutputLogPath("TargetTurretAngle"), setAngle);
    }

    private Translation2d aimTurret(Translation3d target3d) {
        Rotation2d turretAngle = new Rotation2d(target3d.getX(), target3d.getY());
        Logger.recordOutput(getOutputLogPath("TargetTurretAngle"), turretAngle);
        turret.setTargetAngle(turretAngle.getMeasure().plus(initialTurretOffset));
        
        Translation3d targetOnPlane = target3d.rotateBy(new Rotation3d(turretAngle.unaryMinus()));
        if(!(Math.abs(targetOnPlane.getY()) < 1E-6)){
            System.out.println("TargetOnPlane has a Y value of " + targetOnPlane.getY() + ", should be 0");
        }
        return new Translation2d(targetOnPlane.getX(), targetOnPlane.getZ()); //Z is up, is referred to as Y in 2d
    }

    public Command getAimCommand(Translation3d target){
        return new InstantCommand(() -> setTargetFieldSpace(target, null));   
    }
}
