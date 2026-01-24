package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.subsystems.velocity.SparkFlexVelocityIo;
import frc.lib.subsystems.velocity.VelocityRatio;
import frc.lib.subsystems.velocity.VelocitySubsystem;

public class Intake2026 extends VelocitySubsystem {
    private static final int MOTOR_ID = 0; // TODO: Set intake motor ID chubb
    private static final double FORWARDS_VELOCITY_RPM = 300;
    private static final double REVERSE_VELOCITY_RPM = 150;
    
    public Intake2026() {
        super("Intake", new VelocityRatio("GearRatio", 3,
                new SparkFlexVelocityIo("IntakeMotor", new SparkFlex(MOTOR_ID, MotorType.kBrushless), makeConfig())));
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
    }

    public void runReverse() {
        run(REVERSE_VELOCITY_RPM);
    }

}
