package frc.lib.io;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXIoReal extends TalonFXIoBase {

    private final TalonFX motor;
    private final VelocityVoltage velocityControl; 
    private final PositionVoltage positionControl;
    private final VoltageOut voltageControl;

    private final StatusSignal<AngularVelocity> currentVelocity;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> motorTemp;
    private final StatusSignal<Angle> position;

    public TalonFXIoReal(TalonFX motor)  {
        this.motor = motor;

        this.currentVelocity = motor.getVelocity();
        this.statorCurrent = motor.getStatorCurrent();
        this.supplyCurrent = motor.getSupplyCurrent();
        this.motorTemp = motor.getDeviceTemp();
        this.position = motor.getPosition();

        this.velocityControl = new VelocityVoltage(RotationsPerSecond.of(0));
        this.positionControl = new PositionVoltage(Degrees.of(0));
        this.voltageControl = new VoltageOut(Volts.of(0));
    }

    @Override
    protected void updateInputs(TalonFXIoInputs inputs) {
        inputs.currentVelocity = currentVelocity.getValue();
        inputs.statorCurrent = statorCurrent.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.motorTemp = motorTemp.getValue();
        inputs.position = position.getValue();
    }

    @Override
    public void setVelocity(AngularVelocity setpoint) {
        super.setVelocity(setpoint);
        velocityControl.Velocity = setpoint.in(RotationsPerSecond);
        motor.setControl(velocityControl);
    }

    @Override
    public void setAngle(Angle angle) {
        super.setAngle(angle);
        positionControl.Position = angle.in(Rotations);
        motor.setControl(positionControl);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        super.setVoltage(voltage);
        voltageControl.Output = voltage.in(Volts);
        motor.setControl(voltageControl);
    }

    @Override
    public void neutralOutput() {
        super.neutralOutput();
        motor.setControl(new NeutralOut());
    }

    @Override
    public void setPercievedAngle(Angle angle) {
        motor.setPosition(angle); //TODO: Test
    }
}
