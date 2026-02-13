package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.lib.velocity.SparkFlexIo;
import frc.lib.velocity.VelocityRatio;
import frc.lib.velocity.VelocitySubsystem;

public class Indexer2026 extends VelocitySubsystem {
    private static final int MOTOR_ID = 0; // TODO: Set indexer motor ID chubb
    private static final double FORWARDS_VELOCITY_RPM = 90;

    public Indexer2026() {
        super("Indexer", new VelocityRatio("GearRatio", 3,
                new SparkFlexIo("IndexerMotor", new SparkFlex(MOTOR_ID, MotorType.kBrushless), makeConfig())));

    }

    private static SparkFlexConfig makeConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.smartCurrentLimit(80);
        config.closedLoop.pid(0, 0, 0); // TODO: Set these later chubb
        return config;
    }

    public void runForwards() {
        run(FORWARDS_VELOCITY_RPM);
        Logger.recordOutput(getOutputLogPath("TargetVelocity"), FORWARDS_VELOCITY_RPM);
    }

    @Override
    public void stop() {
        super.stop();
        Logger.recordOutput(getOutputLogPath("TargetVelocity"), 0.0);
    }
}
