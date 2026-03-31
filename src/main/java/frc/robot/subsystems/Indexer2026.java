package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Voltage;
import frc.lib.ForwardReverseNeutral;
import frc.lib.velocity.SparkFlexIoBase;
import frc.lib.velocity.SparkFlexIoReal;
import frc.lib.voltage.VoltageComponent;
import frc.lib.voltage.VoltageSubsystem;

public class Indexer2026 extends VoltageSubsystem {
    private static final int MOTOR_ID = 26;
    private static final Voltage FORWARDS_VOLTAGE = Volts.of(6);
    private static final Voltage REVERSE_VOLTAGE = Volts.of(-6);

    private ForwardReverseNeutral commandedState = ForwardReverseNeutral.FORWARD;

    private final BooleanSupplier[] runConditions;

    public Indexer2026(BooleanSupplier... runConditions) {
        super("Indexer", Map.of("IndexerMotor", makeIndexerMotor()));
        this.runConditions = runConditions;
    }

    private static VoltageComponent makeIndexerMotor() {
        VoltageComponent sparkFlex;
        if (Logger.hasReplaySource()) {
            sparkFlex = new SparkFlexIoBase();
        } else {
            SparkFlexConfig config = new SparkFlexConfig();
            config.idleMode(IdleMode.kBrake);
            config.inverted(false);
            config.smartCurrentLimit(80);
            sparkFlex = new SparkFlexIoReal(new SparkFlex(MOTOR_ID, MotorType.kBrushless), config);
        }
        return sparkFlex;
    }

    public void runForwards() {
        commandedState = ForwardReverseNeutral.FORWARD;
    }

    public void runReverse() {
        commandedState = ForwardReverseNeutral.REVERSE;
    }

    public void stop(){
        commandedState = ForwardReverseNeutral.NEUTRAL;
    }

    @Override
    public void neutralOutput() {
        super.neutralOutput();
        stop();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (commandedState == ForwardReverseNeutral.FORWARD) {
            setVoltage(FORWARDS_VOLTAGE);
        } else if (commandedState == ForwardReverseNeutral.REVERSE) {
            setVoltage(REVERSE_VOLTAGE);
        } else {
            boolean run = false;
        for (BooleanSupplier shouldRun : runConditions) {
            if (shouldRun.getAsBoolean()) {
                run = true;
                break;
            }
        }
        if (run) {
            setVoltage(FORWARDS_VOLTAGE);
        } else {
            neutralOutput();
        }
        }
    }
}
