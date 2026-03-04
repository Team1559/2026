package frc.lib.velocity;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.angular_position.AngularPositionComponent;
import frc.lib.loggable.LoggableIo;
import frc.lib.voltage.VoltageComponent;

public class SparkFlexIo extends LoggableIo<SparkFlexIo.SparkFlexIoInputs> implements AngularVelocityComponent, AngularPositionComponent, VoltageComponent {
    @AutoLog
    public static abstract class SparkFlexIoInputs implements LoggableInputs {
        public double motorCurrent;
        public double motorTemp;
        public AngularVelocity currentVelocity;
        public Angle position = Angle.ofRelativeUnits(0, Units.Rotations);
    }

    private final SparkFlex motor;
    private final SparkClosedLoopController motorController;
    private final RelativeEncoder encoder;

    public SparkFlexIo(String name, SparkFlex motor, SparkFlexConfig motorConfig) {
        super(name, new SparkFlexIoInputsAutoLogged());
        this.motor = motor;
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorController = motor.getClosedLoopController();
        encoder = motor.getEncoder();
        Logger.recordOutput(getOutputLogPath("Active"), false);
    }

    @Override
    protected void updateInputs(SparkFlexIoInputs inputs) {
        inputs.currentVelocity = RPM.of(encoder.getVelocity());
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.position = Rotations.of(encoder.getPosition());
    }

    @Override
    public void setVelocity(AngularVelocity setpoint) {
        Logger.recordOutput(getOutputLogPath("VelocitySetpoint"), setpoint);
        motorController.setSetpoint(setpoint.in(Units.RPM), ControlType.kVelocity);
        Logger.recordOutput(getOutputLogPath("Active"), true);
    }

    @Override
    public double getMotorCurrent() {
        return getInputs().motorCurrent;
    }

    @Override
    public double getMotorTemperature() {
        return getInputs().motorTemp;
    }

    @Override
    public AngularVelocity getCurrentVelocity() {
        return getInputs().currentVelocity;
    }

    @Override
    public void setAngle(Angle setpoint) {
        Logger.recordOutput(getOutputLogPath("AngleSetpoint"), setpoint);
        motorController.setSetpoint(setpoint.in(Units.Rotations), ControlType.kPosition);
        Logger.recordOutput(getOutputLogPath("Active"), true);
    }

    @Override
    public Angle getAngle() {
        return getInputs().position;
    }

    @Override
    public void setPercievedAngle(Angle angle) {
        encoder.setPosition(angle.in(Units.Rotations));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
        Logger.recordOutput(getOutputLogPath("Voltage"), voltage.in(Volts));
        Logger.recordOutput(getOutputLogPath("Active"), true);
    }

    @Override
    public void neutralOutput() {
        motor.stopMotor();
        Logger.recordOutput(getOutputLogPath("Active"), false);
    }
}
