package frc.lib.velocity;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

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

public class SparkFlexIoReal extends SparkFlexIoBase {

    private final SparkFlex motor;
    private final SparkClosedLoopController motorController;
    private final RelativeEncoder encoder;

    public SparkFlexIoReal(SparkFlex motor, SparkFlexConfig motorConfig) {
        this.motor = motor;
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorController = motor.getClosedLoopController();
        encoder = motor.getEncoder();
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
