package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.limitSwitch.BooleanComponent;
import frc.lib.limitSwitch.LimitSwitchIo;
import frc.lib.velocity.SparkFlexIo;
import frc.lib.velocity.VelocityRatio;
import frc.lib.velocity.VelocitySubsystem;
import frc.lib.voltage.VoltageComponent;

public class Intake2026 extends VelocitySubsystem {
    private static final int INTAKE_MOTOR_ID = 22;
    private static final int ELBOW_MOTOR_ID = 21;
    private static final double FORWARDS_VELOCITY_RPM = 300;
    private static final double REVERSE_VELOCITY_RPM = -150;

    private final VoltageComponent elbowMotor;
    
    // private final BooleanComponent lowerLimitSwitch;
    // private final BooleanComponent upperLimitSwitch;


    // private final int LOWER_LIMIT_SWITCH_CHANNEL = 0; //TODO add channels
    // private final int UPPER_LIMIT_SWITCH_CHANNEL = 0;

    public Intake2026() {
        super("Intake", new VelocityRatio("GearRatio", 3,
                new SparkFlexIo("IntakeMotor", new SparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless), makeIntakeConfig())));

        elbowMotor = new SparkFlexIo("ElbowMotor", new SparkFlex(ELBOW_MOTOR_ID, MotorType.kBrushless), makeElbowConfig());

        // lowerLimitSwitch = new LimitSwitchIo("LowerLimitSwitch", new DigitalInput(LOWER_LIMIT_SWITCH_CHANNEL));
        // upperLimitSwitch = new LimitSwitchIo("UpperLimitSwitch", new DigitalInput(UPPER_LIMIT_SWITCH_CHANNEL));
    }

    private static SparkFlexConfig makeIntakeConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        config.smartCurrentLimit(80);
        config.closedLoop.pid(0, 0, 0); //TODO: Set these later
        config.closedLoop.feedForward.kV(0.00055);
        return config;
    }

    private static SparkFlexConfig makeElbowConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(80);
        return config;
    }

    public void moveElbowUp() {
        elbowMotor.setVoltage(Volts.of(6));
    }

    public void moveElbowDown() {
        elbowMotor.setVoltage(Volts.of(-6));
    }

    public void stopElbow() {
        elbowMotor.setVoltage(Volts.of(0));
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

    @Override
    public void periodic() {
        super.periodic();
        // lowerLimitSwitch.periodic();
        // upperLimitSwitch.periodic();
    }

    // public boolean isAtUpperLimit() {
    //     return upperLimitSwitch.getAsBoolean();
    // }

    // public boolean isAtLowerLimit() {
    //     return lowerLimitSwitch.getAsBoolean();
    // }

    // public Command downCommand() {
    //     return new FunctionalCommand(() -> moveElbowDown(), () -> {}, (x) -> stopElbow(), () -> isAtLowerLimit(), this);
    // }

    // public Command upCommand() {
    //     return new FunctionalCommand(() -> moveElbowUp(), () -> {}, (x) -> stopElbow(), () -> isAtUpperLimit(), this);
    // }
}
