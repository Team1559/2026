package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.velocity.SparkFlexIo;
import frc.lib.velocity.VelocityRatio;
import frc.lib.velocity.VelocitySubsystem;

public class Intake2026 extends VelocitySubsystem {
    private static final int MOTOR_ID = 0; // TODO: Set intake motor ID chubb
    private static final double FORWARDS_VELOCITY_RPM = 300;
    private static final double REVERSE_VELOCITY_RPM = 150;
    
    public Intake2026() {
        super("Intake", new VelocityRatio("GearRatio", 3,
                new SparkFlexIo("IntakeMotor", new SparkFlex(MOTOR_ID, MotorType.kBrushless), makeConfig())));
    }

    private static SparkFlexConfig makeConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.smartCurrentLimit(80);
        config.closedLoop.pid(0, 0, 0); //TODO: Set these later
        return config;
    }

    public void runForwards() {
        run(FORWARDS_VELOCITY_RPM);
        Logger.recordOutput(getOutputLogPath("TargetVelocity"), FORWARDS_VELOCITY_RPM);
    }

    public void runReverse() {
        run(REVERSE_VELOCITY_RPM);
        Logger.recordOutput(getOutputLogPath("TargetVelocity"), REVERSE_VELOCITY_RPM);
    }

    @Override
    public void stop() {
        super.stop();
        Logger.recordOutput(getOutputLogPath("TargetVelocity"), 0.0);
    }
}
