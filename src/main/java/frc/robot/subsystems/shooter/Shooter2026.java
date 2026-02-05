package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
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
    
    private Translation3d targetFieldSpace;
    private Translation2d barrierOffset;
    private boolean spinFlywheel;
    
    
    private final AngularPositionComponent turret; //APC
    private final AngularPositionSensor turretAngleSensor;
    private final AngularVelocityComponent flywheel; //Velocity Component
    private final AngularPositionComponent flapper; //Angular Position Component
    private final AngularVelocityComponent feedWheel; //VelocityComponent

    private final Angle initialTurretOffset;
    
    private final Pose3d turretOffset;
    //TO​DO: calibrate based on gravity at field/make constants class for venue gravitational accelerations
    public static final double g = 9.80665; //Units: N/kg
    
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
    }

    public Shooter2026(Supplier<Pose2d> robotPositionSupplier){
        this(robotPositionSupplier, new Pose3d(0, 0, 0, Rotation3d.kZero), makeTurret(), makeFlapper(), makeFlywheel(), makeFeedwheel(), makeBaby());//TODO: Offset
    }

    private static SparkFlexIo makeFlywheel(){
        SparkFlexConfig config = new SparkFlexConfig(); //TODO: Configure
        config.closedLoop.pid(0, 0, 0);
        config.closedLoop.feedForward.kV(0);    //Volts per 
        return new SparkFlexIo("Flywheel", new SparkFlex(0, MotorType.kBrushless), config); //TODO: ID motor
    }
    
    private static SparkFlexIo makeFeedwheel(){
        SparkFlexConfig config = new SparkFlexConfig(); //TODO: Configure
        config.closedLoop.pid(0, 0, 0);
        config.closedLoop.feedForward.kV(0);    //Volts per 
        return new SparkFlexIo("Feedwheel", new SparkFlex(0, MotorType.kBrushless), config); //TODO: ID motor
    }

    private static AngularPositionComponent makeTurret() {
        SparkFlexConfig config = new SparkFlexConfig(); //TODO: Configure
        config.closedLoop.pid(0, 0, 0);
        return new LimitedAngularPositionIntermediate("Turret", Degrees.of(-100), Degrees.of(100), new AngularPositionRatio("GearRatio", 30, new SparkFlexIo("TurretMotor", new SparkFlex(0, MotorType.kBrushless), config))); //TODO: ID motor
    }

    private static SparkFlexIo makeFlapper() {
        SparkFlexConfig config = new SparkFlexConfig(); //TODO: Configure
        config.closedLoop.pid(0, 0, 0);
        return new SparkFlexIo("Flapper", new SparkFlex(0, MotorType.kBrushless), config); //TODO: ID motor
    }

    private static AngularPositionSensor makeBaby(){
        CanCoderIo canCoderOne = new CanCoderIo("TurretGearOne", new CANcoder(0), Rotations.of(0)); //TODO: ID motor, calibrate offsets
        CanCoderIo canCoderTwo = new CanCoderIo("TurretGearTwo", new CANcoder(0), Rotations.of(0)); //TODO: ID motor, calibrate offsets
        return new ChineseBaby("ChineseBaby", 21, 19, 200, -9, 10, canCoderOne, canCoderTwo);
    }

    public void setTargetFieldSpace(Translation3d target, Translation2d barrierOffset) {
        this.targetFieldSpace = target;
        this.barrierOffset = barrierOffset;
    }
    
    public void setSpinFlywheel(boolean spinFlywheel) {
        this.spinFlywheel = spinFlywheel;
    }

    private Translation3d calculateTarget() {
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

    private void aimDynamic(Translation3d target3d) {
        Translation2d target = aimTurret(target3d);
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

        double velocityY = Math.sqrt(2 * g * vertexY);
        double timeToApex = velocityY / g;

        double velocityX = vertexX / timeToApex;

        Translation2d projectileVelocity = new Translation2d(velocityX, velocityY);

        flapper.setTargetAngle(projectileVelocity.getAngle().getMeasure());
        if(spinFlywheel){
            double targetAngularVelocity = projectileVelocity.getNorm();    //TODO: create actual equation 
            flywheel.setTargetVelocity(targetAngularVelocity);
        } else {
            flywheel.stop();
        }
    }

    private void aimFixed(Translation3d target3d, Rotation2d angle) {
        Translation2d target = aimTurret(target3d);
        double projectileVelocity =  Math.sqrt(g * (Math.pow(target.getX(), 2) / (2 * Math.pow(angle.getCos(), 2) * (target.getX() * angle.getTan()) - target.getY())));
        if(spinFlywheel){
            double targetAngularVelocity = projectileVelocity;    //TODO: create actual equation 
            flywheel.setTargetVelocity(targetAngularVelocity);
        } else {
            flywheel.stop();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        Translation3d target = calculateTarget();
        if (target != null) {
            aimDynamic(target);
        }
    }

    private Translation2d aimTurret(Translation3d target3d) {
        Rotation2d turretAngle = new Rotation2d(target3d.getX(), target3d.getY());
        turret.setTargetAngle(turretAngle.getMeasure().plus(initialTurretOffset));
        
        Translation3d targetOnPlane = target3d.rotateBy(new Rotation3d(turretAngle.unaryMinus()));
        if(!(Math.abs(targetOnPlane.getY()) < 1E-6)){
            System.out.println("TargetOnPlane has a Y value of " + targetOnPlane.getY() + ", should be 0");
        }
        return new Translation2d(targetOnPlane.getX(), targetOnPlane.getZ()); //Z is up, is referred to as Y in 2d
    }
}
