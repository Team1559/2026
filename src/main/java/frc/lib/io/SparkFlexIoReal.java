package frc.lib.io;

import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;

import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.util.Signal;

public class SparkFlexIoReal extends SparkFlexIoBase {

    private final SparkFlex motor;
    private final SparkClosedLoopController motorController;
    private final RelativeEncoder encoder;

    private final Signal<Double> currentVelocitySupplier;
    private final Signal<Double> motorCurrentSupplier;
    private final Signal<Double> motorTempSupplier;
    private final Signal<Double> positionSupplier;
    private final Signal<Double> appliedOutputSupplier;

    public SparkFlexIoReal(SparkFlex motor, SparkFlexConfig motorConfig) {
        this.motor = motor;
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorController = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        currentVelocitySupplier = encoder.getVelocity();
        motorCurrentSupplier = motor.getOutputCurrent();
        motorTempSupplier = motor.getMotorTemperature();
        positionSupplier = encoder.getPosition();
        appliedOutputSupplier = motor.getAppliedOutput();

    }

    @Override
    protected void updateInputs(SparkFlexIoInputs inputs) {
        inputs.currentVelocity = RPM.of(currentVelocitySupplier.get());
        inputs.motorCurrent = motorCurrentSupplier.get();
        inputs.motorTemp = motorTempSupplier.get();
        inputs.position = Rotations.of(positionSupplier.get());
        inputs.appliedOutput = appliedOutputSupplier.get();
    }

    @Override
    public void setVelocity(AngularVelocity setpoint) {
        super.setVelocity(setpoint);
        motorController.setSetpoint(setpoint.in(Units.RPM), ControlType.kVelocity);
    }

    @Override
    public void setAngle(Angle setpoint) {
        super.setAngle(setpoint);
        motorController.setSetpoint(setpoint.in(Units.Rotations), ControlType.kPosition);
    }

    @Override
    public void setPercievedAngle(Angle angle) {
        super.setPercievedAngle(angle);
        encoder.setPosition(angle.in(Units.Rotations));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        super.setVoltage(voltage);
        motor.setVoltage(voltage);
    }

    @Override
    public void neutralOutput() {
        super.neutralOutput();
        motor.stopMotor();
    }
}
