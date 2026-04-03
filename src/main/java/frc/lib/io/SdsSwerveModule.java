package frc.lib.io;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.lib.component.AngleComponent;
import frc.lib.component.AngleSensor;
import frc.lib.component.DistanceSensor;
import frc.lib.component.LinearVelocityComponent;
import frc.lib.component.SwerveModule;
import frc.lib.logging.LoggableIntermediate;

public class SdsSwerveModule extends LoggableIntermediate implements SwerveModule {

    public enum ModuleType {
        MK4_L1(50d / 14 * 19 / 25 * 45 / 15, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive),
        MK4_L2(50d / 14 * 17 / 27 * 45 / 15, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive),
        MK4_L3(50d / 14 * 16 / 28 * 45 / 15, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive),
        MK4_L4(48d / 16 * 16 / 28 * 45 / 15, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive),

        MK4I_L1(50d / 14 * 19 / 25 * 45 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.Clockwise_Positive),
        MK4I_L2(50d / 14 * 17 / 27 * 45 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.Clockwise_Positive),
        MK4I_L3(50d / 14 * 16 / 28 * 45 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.Clockwise_Positive),

        MK5_R1(54d / 12 * 25 / 32 * 30 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.CounterClockwise_Positive),
        MK5_R2(54d / 14 * 25 / 32 * 30 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.CounterClockwise_Positive),
        MK5_R3(54d / 16 * 25 / 32 * 30 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.CounterClockwise_Positive);

        public final double driveRatio;
        public final InvertedValue driveDirection;
        public final InvertedValue steerDirection;

        ModuleType(double driveRatio, InvertedValue driveDirection, InvertedValue steerDirection) {
            this.driveRatio = driveRatio;
            this.driveDirection = driveDirection;
            this.steerDirection = steerDirection;
        }
    }

    public static final Distance WHEEL_RADIUS = Inches.of(2.0);

    private final AngleComponent steerMotor;
    private final LinearVelocityComponent driveMotor;
    private final DistanceSensor driveMotorDistanceSensor;
    private final AngleSensor encoder;
    // TODO - How can we solve the garbage collection issue with the cancoder?
    // Do we just need to hold on to the AngleSensor like we did with the cancoder
    // before?
    private final Translation2d location;

    public SdsSwerveModule(Translation2d location, AngleComponent steerMotor,
            LinearVelocityComponent driveMotor, DistanceSensor driveMotorDistanceSensor,
            AngleSensor cancoder) {

        this.location = location;
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
        this.driveMotorDistanceSensor = driveMotorDistanceSensor;
        this.encoder = cancoder;
        
        addChild("DriveMotor", driveMotor);
        addChild("DriveMotorDistanceSensor", driveMotorDistanceSensor);
        addChild("SteerMotor", steerMotor);
        addChild("Cancoder", encoder);
    }

    @Override
    public void setSpeed(LinearVelocity speed) {
        driveMotor.setVelocity(speed);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        steerMotor.setAngle(angle);
    }

    @Override
    public void neutralOutput() {
        driveMotor.neutralOutput();
        steerMotor.neutralOutput();
    }

    @Override
    public Translation2d getLocation() {
        return location;
    }

    @Override
    public LinearVelocity getSpeed() {
        return driveMotor.getCurrentVelocity();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(steerMotor.getAngle().in(Rotations));
    }

    @Override
    public Distance getDistance() {
        return driveMotorDistanceSensor.getDistance();
    }
}
