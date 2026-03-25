package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.angular_position.AngularPositionSensor;
import frc.lib.angular_position.CanCoderIo;
import frc.lib.velocity.SparkFlexIo;
import frc.lib.voltage.VoltageComponent;
import frc.lib.voltage.VoltageSubsystem;

public class Intake2026 extends VoltageSubsystem {
    private static final int INTAKE_MOTOR_ID = 22;
    private static final int ELBOW_MOTOR_ID = 21;
    private static final int ELBOW_ENCODER_ID = 16; // TODO: find ID value
    private static final Voltage FORWARD_VOLTAGE = Volts.of(6);
    private static final Voltage REVERSE_VOLTAGE = Volts.of(-5);
    private static final Voltage ELBOW_UP_VOLTAGE = Volts.of(3);
    private static final Voltage ELBOW_DOWN_VOLTAGE = Volts.of(-1);
    private static final Voltage HOLD_ELBOW_UP = Volts.of(0.5);
    private static final Voltage HOLD_ELBOW_DOWN = Volts.of(-0.2);
    private static final Angle ELBOW_OFFSET = Rotations.of(0.757813);
    private static final Angle UP_ANGLE = Degrees.of(86);
    private static final Angle DOWN_ANGLE = Degrees.of(30);

    private final VoltageComponent elbowMotor;
    private final AngularPositionSensor elbowEncoder;

    private ElbowState elbowState = ElbowState.NEUTRAL;
    private IntakeState intakeState = IntakeState.NEUTRAL;
    
    public Intake2026() {
        super("Intake",
                new SparkFlexIo("IntakeMotor", new SparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless),
                        makeIntakeConfig()));

        elbowMotor = new SparkFlexIo("ElbowMotor", new SparkFlex(ELBOW_MOTOR_ID, MotorType.kBrushless),
                makeElbowConfig());

        elbowEncoder = new CanCoderIo("ElbowEncoder", new CANcoder(ELBOW_ENCODER_ID), ELBOW_OFFSET,
                makeEncoderConfig());

        addChildren(elbowMotor, elbowEncoder);
    }

    private static SparkFlexConfig makeIntakeConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.smartCurrentLimit(80);
        return config;
    }

    private static SparkFlexConfig makeElbowConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(80);
        return config;
    }

    private static CANcoderConfiguration makeEncoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        return config;
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
        intakeState = IntakeState.FOWARDS;
    }

    public void runReverse() {
        intakeState = IntakeState.BACKWARDS;
    }

    @Override
    public void neutralOutput() {
        intakeState = IntakeState.NEUTRAL;
    }

    @Override
    public void periodic() {
        super.periodic();
        logger().debug("ElbowUp", isAtUpperLimit())
                .debug("ElbowDown", isAtLowerLimit())
                .debug("ElbowState", elbowState);

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
                super.neutralOutput();
                break;
            case FOWARDS:
                if (isAtLowerLimit()) {
                    setVoltage(FORWARD_VOLTAGE);
                } else {
                    super.neutralOutput();
                }
                break;
            case BACKWARDS:
                if (isAtLowerLimit()) {
                    setVoltage(REVERSE_VOLTAGE);
                } else {
                    super.neutralOutput();
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

    public boolean isIntaking(){
        return intakeState == IntakeState.FOWARDS;
    }

    private enum ElbowState {
        UP,
        DOWN,
        NEUTRAL
    }

    private enum IntakeState{
        FOWARDS,
        BACKWARDS,
        NEUTRAL,
    }
}
