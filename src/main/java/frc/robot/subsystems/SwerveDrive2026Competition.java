package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import frc.lib.component.AngleComponent;
import frc.lib.component.AngleSensor;
import frc.lib.component.SwerveModule;
import frc.lib.intermediate.DriveSteerSwerveModule;
import frc.lib.intermediate.DriveWheelAdapter;
import frc.lib.io.CanCoderIoBase;
import frc.lib.io.CanCoderIoReal;
import frc.lib.io.Pigeon2IoBase;
import frc.lib.io.Pigeon2IoReal;
import frc.lib.io.TalonFXIoBase;
import frc.lib.io.TalonFXIoReal;
import frc.lib.subsystem.SwerveDrive;
import frc.lib.util.SdsSwerveModuleType;

public class SwerveDrive2026Competition extends SwerveDrive {
    private static final CANBus CANIVORE_BUS = new CANBus("1559_Canivore");
    private static final Mass MASS = Pounds.of(104);
    private static final Distance RADIUS = Inches.of(27 / 2.0); // Give or take
    private static final MomentOfInertia MOI = KilogramSquareMeters
            .of(MASS.in(Kilograms) * RADIUS.in(Meters) * RADIUS.in(Meters));

    private static final LinearVelocity SWERVE_MAX_LINEAR_VELOCITY = MetersPerSecond.of(5);
    private static final LinearAcceleration SWERVE_MAX_LINEAR_ACCEL = SWERVE_MAX_LINEAR_VELOCITY.div(Seconds.of(1));
    private static final AngularVelocity SWERVE_MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(2);
    private static final AngularAcceleration SWERVE_MAX_ANGULAR_ACCEL = SWERVE_MAX_ANGULAR_VELOCITY.div(Seconds.of(.5));

    private static final AngularVelocity KRAKEN_MAX_FREE_VELOCITY = RPM.of(6000.0);
    private static final Voltage BATTERY_VOLTAGE = Volts.of(12.0);
    private static final Current DRIVE_MOTOR_STATOR_CURRENT = Amps.of(60.0);
    private static final Current DRIVE_MOTOR_SUPPLY_CURRENT = Amps.of(40.0);
    private static final double COEFFICENT_OF_FRICTION = 0.5;

    private static final SdsSwerveModuleType MODULE_TYPE = SdsSwerveModuleType.MK5_R2;

    public static final SwerveConstraints SWERVE_CONSTRAINTS = new SwerveConstraints(SWERVE_MAX_ANGULAR_VELOCITY,
            SWERVE_MAX_ANGULAR_ACCEL, SWERVE_MAX_LINEAR_VELOCITY, SWERVE_MAX_LINEAR_ACCEL);
    public static final SwerveConstraints SLOW_SWERVE_CONSTRAINTS = new SwerveConstraints(
            SWERVE_MAX_ANGULAR_VELOCITY.div(6.0), SWERVE_MAX_ANGULAR_ACCEL, SWERVE_MAX_LINEAR_VELOCITY.div(6),
            SWERVE_MAX_LINEAR_ACCEL);

    public SwerveDrive2026Competition() {
        super("SwerveDrive", createGyro(), createModules());

        SwerveModule[] modules = getModules();
        Translation2d[] locations = new Translation2d[modules.length];
        for (int i = 0; i < locations.length; i++) {
            locations[i] = modules[i].getLocation();
        }
        RobotConfig config = new RobotConfig(MASS, MOI,
                new ModuleConfig(SdsSwerveModuleType.WHEEL_RADIUS.in(Meters),
                        SWERVE_MAX_LINEAR_VELOCITY.in(MetersPerSecond),
                        COEFFICENT_OF_FRICTION,
                        DCMotor.getKrakenX60(1)
                                .withReduction(Math.abs(MODULE_TYPE.driveRatio)),
                        DRIVE_MOTOR_STATOR_CURRENT.in(Amps),
                        1),
                locations);
        configureAuto(config);
    }

    @Override
    public void periodic() {
        super.periodic();
        logger().debug("CAN Utilization", CANIVORE_BUS.getStatus().BusUtilization);
    }

    private static DriveSteerSwerveModule createSwerveModule(int steerMotorId, int driveMotorId, int canCoderId,
            Rotation2d canCoderOffset, Translation2d locationOffset) {

        AngleComponent steerMotor;
        AngleSensor encoder;

        TalonFXIoBase driveMotorIO;
        if (Logger.hasReplaySource()) {
            steerMotor = new TalonFXIoBase();
            driveMotorIO = new TalonFXIoBase();
            encoder = new CanCoderIoBase();
        } else {
            CANcoder canCoder = new CANcoder(canCoderId, CANIVORE_BUS);
            encoder = new CanCoderIoReal(canCoder, new CANcoderConfiguration());
            TalonFX steerMotorTalonFX = new TalonFX(steerMotorId, CANIVORE_BUS);
            steerMotorTalonFX.getConfigurator().apply(new TalonFXConfiguration());
            steerMotorTalonFX.getConfigurator().apply(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(MODULE_TYPE.steerDirection));
            // Apply PID Config
            steerMotorTalonFX.getConfigurator().apply(new Slot0Configs().withKP(80));
            steerMotorTalonFX.getConfigurator().apply(new FeedbackConfigs().withRemoteCANcoder(canCoder));
            ClosedLoopGeneralConfigs clgConfig = new ClosedLoopGeneralConfigs();
            clgConfig.ContinuousWrap = true;
            steerMotorTalonFX.getConfigurator().apply(clgConfig);
            steerMotor = new TalonFXIoReal(steerMotorTalonFX).withOffset(canCoderOffset);

            TalonFX driveMotorTalonFX = new TalonFX(driveMotorId, CANIVORE_BUS);
            driveMotorTalonFX.getConfigurator().apply(new TalonFXConfiguration());
            driveMotorTalonFX.getConfigurator().apply(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(MODULE_TYPE.driveDirection));
            driveMotorTalonFX.getConfigurator().apply(new Slot0Configs()
                    .withKV(BATTERY_VOLTAGE.in(Volts) / (KRAKEN_MAX_FREE_VELOCITY.in(RotationsPerSecond))));
            driveMotorTalonFX.getConfigurator()
                    .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(DRIVE_MOTOR_STATOR_CURRENT)
                            .withSupplyCurrentLimit(DRIVE_MOTOR_SUPPLY_CURRENT));
            driveMotorTalonFX.setPosition(0);
            driveMotorIO = new TalonFXIoReal(driveMotorTalonFX);
        }

        DriveWheelAdapter<?> driveMotor = new DriveWheelAdapter<>(driveMotorIO, SdsSwerveModuleType.WHEEL_RADIUS,
                MODULE_TYPE.driveRatio);

        return new DriveSteerSwerveModule(locationOffset, steerMotor,
                driveMotor);
    }

    private static Pigeon2IoBase createGyro() {
        return new Pigeon2IoReal(new Pigeon2(13, CANIVORE_BUS));
    }

    private static Map<String, SwerveModule> createModules() {
        double swerveModuleX = Units.inchesToMeters(10.875);
        double swerveModuleY = Units.inchesToMeters(10.875);
        DriveSteerSwerveModule frontLeft = createSwerveModule(1, 3, 2, Rotation2d.fromRadians(1.866855),
                new Translation2d(swerveModuleX, swerveModuleY));
        DriveSteerSwerveModule frontRight = createSwerveModule(4, 6, 5, Rotation2d.fromRadians(1.825437),
                new Translation2d(swerveModuleX, -swerveModuleY));
        DriveSteerSwerveModule rearLeft = createSwerveModule(10, 12, 11, Rotation2d.fromRadians(-0.248505),
                new Translation2d(-swerveModuleX, swerveModuleY));
        DriveSteerSwerveModule rearRight = createSwerveModule(7, 9, 8, Rotation2d.fromRadians(-0.509282),
                new Translation2d(-swerveModuleX, -swerveModuleY));
        return Map.of("FrontLeft", frontLeft, "FrontRight", frontRight, "RearLeft", rearLeft, "RearRight", rearRight);
    }
}
