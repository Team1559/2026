package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Voltage;
import frc.lib.velocity.SparkFlexIo;
import frc.lib.voltage.VoltageSubsystem;

public class Indexer2026 extends VoltageSubsystem {
    private static final int MOTOR_ID = 26;
    private static final Voltage FORWARDS_VOLTAGE = Volts.of(6);

    public Indexer2026() {
        super("Indexer", new SparkFlexIo("IndexerMotor", new SparkFlex(MOTOR_ID, MotorType.kBrushless), makeConfig()));
    }

    private static SparkFlexConfig makeConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.smartCurrentLimit(80);
        return config;
    }

    public void runForwards() {
        run(FORWARDS_VOLTAGE);
        Logger.recordOutput(getOutputLogPath("TargetVoltage"), FORWARDS_VOLTAGE);
    }

    @Override
    public void stop() {
        super.stop();
        Logger.recordOutput(getOutputLogPath("TargetVoltage"), 0.0);
    }

}
