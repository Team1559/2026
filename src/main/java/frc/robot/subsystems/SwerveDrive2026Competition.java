package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.swerve.GyroIo;
import frc.lib.swerve.Pigeon2Io;
import frc.lib.swerve.SdsSwerveModuleIo;
import frc.lib.swerve.SdsSwerveModuleIo.ModuleType;
import frc.lib.swerve.SwerveDrive;
import frc.lib.swerve.SwerveModule;
import frc.lib.swerve.SwerveModuleIo;

public class SwerveDrive2026Competition extends SwerveDrive {
    private static final CANBus CANIVORE_BUS = new CANBus("1559_Canivore");
    private static final double MASS = Units.lbsToKilograms(50); //TODO: change the mass
    private static final double RADIUS = Units.inchesToMeters(27 / 2.0); // Give or take
    private static final double MOI = MASS * RADIUS * RADIUS;
    private static final double SWERVE_MAX_LINEAR_VELOCITY = 5;
    private static final double SWERVE_MAX_LINEAR_ACCEL = 5 / 1.6667;
    private static final double SWERVE_MAX_ANGULAR_VELOCITY = 12;
    private static final double KRAKEN_MAX_FREE_VELOCITY = 6000.0;
    private static final double BATTERY_VOLTAGE = 12.0;
    private static final double SECONDS_PER_MINUTE = 60.0;
    private static final double DRIVE_MOTOR_CURRENT = 80.0;

    private static final double SWERVE_MAX_ANGULAR_ACCEL = SWERVE_MAX_ANGULAR_VELOCITY / 0.5;
    public static final SwerveConstraints SWERVE_CONSTRAINTS = new SwerveConstraints(SWERVE_MAX_ANGULAR_VELOCITY,
            SWERVE_MAX_ANGULAR_ACCEL, SWERVE_MAX_LINEAR_VELOCITY, SWERVE_MAX_LINEAR_ACCEL);
    public static final SwerveConstraints SLOW_SWERVE_CONSTRAINTS = new SwerveConstraints(
            SWERVE_MAX_ANGULAR_VELOCITY / 6, SWERVE_MAX_ANGULAR_ACCEL, SWERVE_MAX_LINEAR_VELOCITY / 6,
            SWERVE_MAX_LINEAR_ACCEL);

    public SwerveDrive2026Competition() {
        super("SwerveDrive", createGyro(), createModules());

        SwerveModule[] modules = getModules();
        Translation2d[] locations = new Translation2d[modules.length];
        for (int i = 0; i < locations.length; i++) {
            locations[i] = modules[i].getLocation();
        }
        RobotConfig config = new RobotConfig(MASS, MOI,
                new ModuleConfig(SdsSwerveModuleIo.WHEEL_RADIUS, SWERVE_MAX_LINEAR_VELOCITY, 1.0,
                        DCMotor.getKrakenX60(1).withReduction(Math.abs(SdsSwerveModuleIo.ModuleType.MK5_R2.driveRatio)), DRIVE_MOTOR_CURRENT,
                        1),
                locations);
        configureAuto(config);
    }

    @Override
    public void periodic() {
        super.periodic();
        Logger.recordOutput(getOutputLogPath("CAN Utilization"), CANIVORE_BUS.getStatus().BusUtilization);
    }

    private static SwerveModuleIo createSwerveModule(String name, int steerMotorId, int driveMotorId,
            int canCoderId,
            Rotation2d canCoderOffset, Translation2d locationOffset) {

        CANcoder canCoder = new CANcoder(canCoderId, CANIVORE_BUS);
        TalonFX steerMotor = new TalonFX(steerMotorId, CANIVORE_BUS);
        TalonFX driveMotor = new TalonFX(driveMotorId, CANIVORE_BUS);

        Slot0Configs steerMotorPid = new Slot0Configs().withKP(80);
        Slot0Configs driveMotorPid = new Slot0Configs().withKV(BATTERY_VOLTAGE / (KRAKEN_MAX_FREE_VELOCITY / SECONDS_PER_MINUTE));

        return new SdsSwerveModuleIo(name, locationOffset, ModuleType.MK5_R2, steerMotor, steerMotorPid,
                driveMotor,
                driveMotorPid, DRIVE_MOTOR_CURRENT,
                canCoder, canCoderOffset);
    }

    private static GyroIo createGyro() {
        return new Pigeon2Io("Gyro", new Pigeon2(13, CANIVORE_BUS));
    }

    private static SwerveModuleIo[] createModules() {
        double swerveModuleX = Units.inchesToMeters(10.875);
        double swerveModuleY = Units.inchesToMeters(10.875);
        SwerveModuleIo frontLeft = createSwerveModule("frontLeft", 1, 3, 2, Rotation2d.fromRadians(1.862253),
                new Translation2d(swerveModuleX, swerveModuleY));
        SwerveModuleIo frontRight = createSwerveModule("frontRight", 4, 6, 5, Rotation2d.fromRadians(1.814699),
                new Translation2d(swerveModuleX, -swerveModuleY));
        SwerveModuleIo rearLeft = createSwerveModule("rearLeft", 10, 12, 11, Rotation2d.fromRadians(-0.248505),
                new Translation2d(-swerveModuleX, swerveModuleY));
        SwerveModuleIo rearRight = createSwerveModule("rearRight", 7, 9, 8, Rotation2d.fromRadians(-0.49701),
                new Translation2d(-swerveModuleX, -swerveModuleY));
        return new SwerveModuleIo[] { frontLeft, frontRight, rearLeft, rearRight };
    }
}
