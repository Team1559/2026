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

public class SwerveDrive2026 extends SwerveDrive {
    private static final CANBus CANIVORE_BUS = new CANBus("1559_Canivore");
    private static final double MASS = Units.lbsToKilograms(50);
    private static final double RADIUS = Units.inchesToMeters(27 / 2.0); // Give or take
    private static final double MOI = MASS * RADIUS * RADIUS;
    private static final double SWERVE_MAX_LINEAR_VELOCITY = 5.21;
    private static final double SWERVE_MAX_LINEAR_ACCEL = 3;
    private static final double SWERVE_MAX_ANGULAR_VELOCITY = 18;

    private static final double SWERVE_MAX_ANGULAR_ACCEL = SWERVE_MAX_ANGULAR_VELOCITY / .01;
    public static final SwerveConstraints SWERVE_CONSTRAINTS = new SwerveConstraints(SWERVE_MAX_ANGULAR_VELOCITY,
            SWERVE_MAX_ANGULAR_ACCEL, SWERVE_MAX_LINEAR_VELOCITY, SWERVE_MAX_LINEAR_ACCEL);
    public static final SwerveConstraints SLOW_SWERVE_CONSTRAINTS = new SwerveConstraints(
            SWERVE_MAX_ANGULAR_VELOCITY / 6, SWERVE_MAX_ANGULAR_ACCEL, SWERVE_MAX_LINEAR_VELOCITY / 6,
            SWERVE_MAX_LINEAR_ACCEL);

    public SwerveDrive2026() {
        super("SwerveDrive", createGyro(), createModules());

        SwerveModule[] modules = getModules();
        Translation2d[] locations = new Translation2d[modules.length];
        for (int i = 0; i < locations.length; i++) {
            locations[i] = modules[i].getLocation();
        }
        RobotConfig config = new RobotConfig(MASS, MOI,
                new ModuleConfig(SdsSwerveModuleIo.WHEEL_RADIUS, 5, 1.0,
                        DCMotor.getKrakenX60(1).withReduction(50d / 14 * 16 / 28 * 45 / 15), 80.0,
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
        Slot0Configs driveMotorPid = new Slot0Configs().withKV(12 / (6380.0 / 60)); // TODO: add the kd

        return new SdsSwerveModuleIo(name, locationOffset, ModuleType.MK4i_L3, steerMotor, steerMotorPid,
                driveMotor,
                driveMotorPid,
                canCoder, canCoderOffset);
    }

    private static GyroIo createGyro() {
        return new Pigeon2Io("Gyro", new Pigeon2(13, CANIVORE_BUS));
    }

    private static SwerveModuleIo[] createModules() { // TODO: calibrate the cancoder offset
        double swerveModuleX = Units.inchesToMeters(10.875);
        double swerveModuleY = Units.inchesToMeters(10.875);
        SwerveModuleIo frontLeft = createSwerveModule("frontLeft", 1, 3, 2, Rotation2d.fromRadians(-0.904),
                new Translation2d(swerveModuleX, swerveModuleY));
        SwerveModuleIo frontRight = createSwerveModule("frontRight", 4, 6, 5, Rotation2d.fromRadians(1.729),
                new Translation2d(swerveModuleX, -swerveModuleY));
        SwerveModuleIo rearLeft = createSwerveModule("rearLeft", 10, 12, 11, Rotation2d.fromRadians(-3.077),
                new Translation2d(-swerveModuleX, swerveModuleY));
        SwerveModuleIo rearRight = createSwerveModule("rearRight", 7, 9, 8, Rotation2d.fromRadians(0.873),
                new Translation2d(-swerveModuleX, -swerveModuleY));
        return new SwerveModuleIo[] { frontLeft, frontRight, rearLeft, rearRight };
    }
}
