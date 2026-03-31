package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import frc.lib.component.SwerveModule;
import frc.lib.io.Pigeon2IoBase;
import frc.lib.io.Pigeon2IoReal;
import frc.lib.io.SdsSwerveModuleIoBase;
import frc.lib.io.SdsSwerveModuleIoReal;
import frc.lib.io.SdsSwerveModuleIoReal.ModuleType;
import frc.lib.subsystem.SwerveDrive;

public class SwerveDrive2026Practice extends SwerveDrive {
    private static final CANBus CANIVORE_BUS = new CANBus("1559_Canivore");
    private static final double MASS = Units.lbsToKilograms(50);
    private static final double RADIUS = Units.inchesToMeters(27 / 2.0); // Give or take
    private static final double MOI = MASS * RADIUS * RADIUS;

    private static final LinearVelocity SWERVE_MAX_LINEAR_VELOCITY = MetersPerSecond.of(5);
    private static final LinearAcceleration SWERVE_MAX_LINEAR_ACCEL = SWERVE_MAX_LINEAR_VELOCITY.div(Seconds.of(1));
    private static final AngularVelocity SWERVE_MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(2);
    private static final AngularAcceleration SWERVE_MAX_ANGULAR_ACCEL = SWERVE_MAX_ANGULAR_VELOCITY.div(Seconds.of(.5));

    private static final double KRAKEN_MAX_FREE_VELOCITY = 6000.0;
    private static final double BATTERY_VOLTAGE = 12.0;
    private static final double SECONDS_PER_MINUTE = 60.0;
    private static final double DRIVE_MOTOR_STATOR_CURRENT = 60.0;
    private static final double DRIVE_MOTOR_SUPPLY_CURRENT = 40.0;

    public static final SwerveConstraints SWERVE_CONSTRAINTS = new SwerveConstraints(SWERVE_MAX_ANGULAR_VELOCITY,
            SWERVE_MAX_ANGULAR_ACCEL, SWERVE_MAX_LINEAR_VELOCITY, SWERVE_MAX_LINEAR_ACCEL);
    public static final SwerveConstraints SLOW_SWERVE_CONSTRAINTS = new SwerveConstraints(
            SWERVE_MAX_ANGULAR_VELOCITY.div(6), SWERVE_MAX_ANGULAR_ACCEL, SWERVE_MAX_LINEAR_VELOCITY.div(6),
            SWERVE_MAX_LINEAR_ACCEL);

    public SwerveDrive2026Practice() {
        super("SwerveDrive", createGyro(), createModules());

        SwerveModule[] modules = getModules();
        Translation2d[] locations = new Translation2d[modules.length];
        for (int i = 0; i < locations.length; i++) {
            locations[i] = modules[i].getLocation();
        }
        RobotConfig config = new RobotConfig(MASS, MOI,
                new ModuleConfig(SdsSwerveModuleIoReal.WHEEL_RADIUS, SWERVE_MAX_LINEAR_VELOCITY.in(MetersPerSecond),
                        1.0,
                        DCMotor.getKrakenX60(1)
                                .withReduction(Math.abs(SdsSwerveModuleIoReal.ModuleType.MK4I_L3.driveRatio)),
                        DRIVE_MOTOR_STATOR_CURRENT,
                        1),
                locations);
        configureAuto(config);
    }

    @Override
    public void periodic() {
        super.periodic();
        logger().debug("CAN Utilization", CANIVORE_BUS.getStatus().BusUtilization);
    }

    private static SdsSwerveModuleIoBase createSwerveModule(int steerMotorId, int driveMotorId, int canCoderId,
            Rotation2d canCoderOffset, Translation2d locationOffset) {

        if (Logger.hasReplaySource()) {
            return new SdsSwerveModuleIoBase(locationOffset);
        }

        CANcoder canCoder = new CANcoder(canCoderId, CANIVORE_BUS);
        TalonFX steerMotor = new TalonFX(steerMotorId, CANIVORE_BUS);
        TalonFX driveMotor = new TalonFX(driveMotorId, CANIVORE_BUS);

        Slot0Configs steerMotorPid = new Slot0Configs().withKP(80);
        Slot0Configs driveMotorPid = new Slot0Configs()
                .withKV(BATTERY_VOLTAGE / (KRAKEN_MAX_FREE_VELOCITY / SECONDS_PER_MINUTE));

        return new SdsSwerveModuleIoReal(locationOffset, ModuleType.MK4I_L3, steerMotor, steerMotorPid,
                driveMotor,
                driveMotorPid, DRIVE_MOTOR_STATOR_CURRENT, DRIVE_MOTOR_SUPPLY_CURRENT,
                canCoder, canCoderOffset);
    }

    private static Pigeon2IoBase createGyro() {
        return new Pigeon2IoReal(new Pigeon2(13, CANIVORE_BUS));
    }

    private static Map<String, SwerveModule> createModules() {
        double swerveModuleX = Units.inchesToMeters(10.875);
        double swerveModuleY = Units.inchesToMeters(10.875);
        SdsSwerveModuleIoBase frontLeft = createSwerveModule(1, 3, 2, Rotation2d.fromRadians(-0.904),
                new Translation2d(swerveModuleX, swerveModuleY));
        SdsSwerveModuleIoBase frontRight = createSwerveModule(4, 6, 5, Rotation2d.fromRadians(1.729),
                new Translation2d(swerveModuleX, -swerveModuleY));
        SdsSwerveModuleIoBase rearLeft = createSwerveModule(10, 12, 11, Rotation2d.fromRadians(-3.077),
                new Translation2d(-swerveModuleX, swerveModuleY));
        SdsSwerveModuleIoBase rearRight = createSwerveModule(7, 9, 8, Rotation2d.fromRadians(0.873),
                new Translation2d(-swerveModuleX, -swerveModuleY));
        return Map.of("FrontLeft", frontLeft, "FrontRight", frontRight, "RearLeft", rearLeft, "RearRight", rearRight);
    }
}
