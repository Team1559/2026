package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.lib.subsystems.velocity.SparkFlexVelocityIo;
import frc.lib.subsystems.velocity.VelocityRatio;
import frc.lib.subsystems.velocity.VelocitySubsystem;

public class Indexer2026 extends VelocitySubsystem {
    private static final int MOTOR_ID = 0; // TODO: Set indexer motor ID chubb
    private static final double FORWARDS_VELOCITY_RPM = 90;

    public Indexer2026() {
        super("Indexer", new VelocityRatio("GearRatio", 3,
                new SparkFlexVelocityIo("IndexerMotor", new SparkFlex(MOTOR_ID, MotorType.kBrushless), makeConfig())));

    }

    private static SparkFlexConfig makeConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.smartCurrentLimit(80);
        config.closedLoop.pid(0, 0, 0); // TODO: Set these later
        return config;
    }

    public void runForwards() {
        run(FORWARDS_VELOCITY_RPM);
    }
}
