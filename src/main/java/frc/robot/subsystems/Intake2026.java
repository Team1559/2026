package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.lib.component.AngleSensor;
import frc.lib.component.VoltageComponent;
import frc.lib.io.CanCoderIoBase;
import frc.lib.io.CanCoderIoReal;
import frc.lib.io.SparkFlexIoBase;
import frc.lib.io.SparkFlexIoReal;
import frc.lib.logging.LoggableSubsystem;
import frc.lib.util.ForwardReverseNeutral;

public class Intake2026 extends LoggableSubsystem {
    private static final int INTAKE_MOTOR_ID = 22;
    private static final int ELBOW_MOTOR_ID = 21;
    private static final int ELBOW_ENCODER_ID = 16;
    private static final Voltage FORWARD_VOLTAGE = Volts.of(6);
    private static final Voltage REVERSE_VOLTAGE = Volts.of(-5);
    private static final Voltage ELBOW_UP_VOLTAGE = Volts.of(3);
    private static final Voltage ELBOW_DOWN_VOLTAGE = Volts.of(-1);
    private static final Voltage HOLD_ELBOW_UP = Volts.of(0.5);
    private static final Voltage HOLD_ELBOW_DOWN = Volts.of(-0.2);
    private static final Angle ELBOW_OFFSET = Radians.of(4.64336);
    private static final Angle UP_ANGLE = Degrees.of(86);
    private static final Angle DOWN_ANGLE = Degrees.of(30);

    private final VoltageComponent intakeMotor;
    private final VoltageComponent elbowMotor;
    private final AngleSensor elbowEncoder;
    
    private ElbowState elbowState = ElbowState.NEUTRAL;
    private ForwardReverseNeutral intakeState = ForwardReverseNeutral.NEUTRAL;

    public Intake2026() {
        super("Intake");

        intakeMotor = makeIntakeMotor();
        elbowMotor = makeElbowMotor();
        elbowEncoder = makeElbowEncoder();

        addChild("IntakeMotor", intakeMotor);
        addChild("ElbowMotor", elbowMotor);
        addChild("ElbowEncoder", elbowEncoder);
    }

    private static VoltageComponent makeIntakeMotor() {
        VoltageComponent sparkFlex;
        if (Logger.hasReplaySource() ) {
            sparkFlex = new SparkFlexIoBase();
        } else {
            SparkFlexConfig config = new SparkFlexConfig();
            config.idleMode(IdleMode.kBrake);
            config.inverted(false);
            config.smartCurrentLimit(80);
            sparkFlex = new SparkFlexIoReal(new SparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless), config);
        }
        return sparkFlex;
    }

    private static AngleSensor makeElbowEncoder() {
        AngleSensor encoder;
        if (Logger.hasReplaySource()) {
            encoder = new CanCoderIoBase();
        } else {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
            encoder = new CanCoderIoReal(new CANcoder(ELBOW_ENCODER_ID), config);
        }
        return encoder.withOffset(ELBOW_OFFSET);
    }

    private static VoltageComponent makeElbowMotor() {
        VoltageComponent sparkFlex;
        if (Logger.hasReplaySource()) {
            sparkFlex = new SparkFlexIoBase();
        } else {
            SparkFlexConfig config = new SparkFlexConfig();
            config.idleMode(IdleMode.kBrake);
            config.smartCurrentLimit(80);
            sparkFlex = new SparkFlexIoReal(new SparkFlex(ELBOW_MOTOR_ID, MotorType.kBrushless), config);
        }
        return sparkFlex;
    }

    public void moveElbowUp() {
        elbowState = ElbowState.UP;
    }

    public void moveElbowDown() {
        elbowState = ElbowState.DOWN;
    }

    public void elbowNeutral() {
        elbowState = ElbowState.NEUTRAL;
    }

    public void runForwards() {
        intakeState = ForwardReverseNeutral.FORWARD;
    }

    public void runReverse() {
        intakeState = ForwardReverseNeutral.REVERSE;
    }

    public void stop() {
        intakeState = ForwardReverseNeutral.NEUTRAL;
    }

    @Override
    public void periodic() {
        super.periodic();
        logger().debug("ElbowUp", isAtUpperLimit())
                .debug("ElbowDown", isAtLowerLimit())
                .debug("ElbowState", elbowState)
                .debug("ElbowAngle", elbowEncoder.getAngle());
        switch (elbowState) {
            case NEUTRAL:
                elbowMotor.neutralOutput();
                break;
            case UP:
                if (isAtUpperLimit()) {
                    elbowMotor.setVoltage(HOLD_ELBOW_UP);
                } else {
                    elbowMotor.setVoltage(ELBOW_UP_VOLTAGE);
                }
                break;
            case DOWN:
                if (isAtLowerLimit()) {
                    elbowMotor.setVoltage(HOLD_ELBOW_DOWN);
                } else {
                    elbowMotor.setVoltage(ELBOW_DOWN_VOLTAGE);
                }
                break;
        }

        switch (intakeState) {
            case NEUTRAL:
                intakeMotor.neutralOutput();
                break;
            case FORWARD:
                if (isAtLowerLimit()) {
                    intakeMotor.setVoltage(FORWARD_VOLTAGE);
                } else {
                    intakeMotor.neutralOutput();
                }
                break;
            case REVERSE:
                if (isAtLowerLimit()) {
                    intakeMotor.setVoltage(REVERSE_VOLTAGE);
                } else {
                    intakeMotor.neutralOutput();
                }
                break;
        }
    }

    public boolean isAtUpperLimit() {
        return elbowEncoder.getAngle().gte(UP_ANGLE);
    }

    public boolean isAtLowerLimit() {
        return elbowEncoder.getAngle().lte(DOWN_ANGLE);
    }

    public boolean isIntaking() {
        return intakeState == ForwardReverseNeutral.FORWARD;
    }

    private enum ElbowState {
        UP,
        DOWN,
        NEUTRAL
    }
}
