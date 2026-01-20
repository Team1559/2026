package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.subsystems.LoggableSubsystem;

public class Shooter2026 extends LoggableSubsystem {
    private final Supplier<Pose2d> robotPositionSupplier;
    
    private Translation3d targetFieldSpace;
    private Translation2d barrierOffset;
    private boolean spinFlywheel;
    
    
    private final TurretIo2026 turret;
    private final FlywheelIo2026 flywheel;
    private final FlapperIo2026 flapper;
    
    private final Pose3d turretOffset;
    //TO​DO: calibrate based on gravity at field/make constants class for venue gravitational accelerations
    public static final double ACCELERATION_DUE_TO_GRAVITY_ON_EARTH_AT_SEA_LEVEL = 9.806; //Units: N/kg
    
    public Shooter2026(Supplier<Pose2d> robotPositionSupplier, Pose3d turretOffset, TurretIo2026 turret, FlywheelIo2026 flywheel, FlapperIo2026 flapper) {
        super("Shooter");
        this.robotPositionSupplier = robotPositionSupplier;
        this.turretOffset = turretOffset;
        this.turret = turret;
        this.flywheel = flywheel;
        this.flapper = flapper;
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
    private void aim(Translation3d target3d) {
        Rotation2d turretAngle = new Rotation2d(target3d.getX(), target3d.getY());
        turret.setTarget(turretAngle);
        
        Translation3d targetOnPlane = target3d.rotateBy(new Rotation3d(turretAngle.unaryMinus()));
        if(!(Math.abs(targetOnPlane.getY()) < 1E-6)){
            System.out.println("TargetOnPlane has a Y value of " + targetOnPlane.getY() + ", should be 0");
        }
        Translation2d target = new Translation2d(targetOnPlane.getX(), targetOnPlane.getZ()); //Z is up, is referred to as Y in 2d
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

        double velocityY = Math.sqrt(2 * ACCELERATION_DUE_TO_GRAVITY_ON_EARTH_AT_SEA_LEVEL * vertexY);
        double timeToApex = velocityY / ACCELERATION_DUE_TO_GRAVITY_ON_EARTH_AT_SEA_LEVEL;

        double velocityX = vertexX / timeToApex;

        Translation2d projectileVelocity = new Translation2d(velocityX, velocityY);

        flapper.setTargetProjectileAngle(projectileVelocity.getAngle());
        if(spinFlywheel){
            flywheel.setProjectileVelocity(projectileVelocity.getNorm());
        } else {
            flywheel.stop();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        Translation3d target = calculateTarget();
        if (target != null) {
            aim(target);
        }
    }
}
