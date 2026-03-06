package frc.lib.voltage;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import frc.lib.NeutralOutput;
import frc.lib.loggable.LoggableSubsystem;

public class VoltageSubsystem extends LoggableSubsystem implements NeutralOutput {
    private final VoltageComponent[] children;
    private Voltage setpoint = Volts.zero();

    public VoltageSubsystem(String name, VoltageComponent... children) {
        super(name);
        this.children = children;
        addChildren(children);
    }

    public void setVoltage(Voltage setpoint) {
        this.setpoint = setpoint;
        Logger.recordOutput(getOutputLogPath("VoltageSetpoint"), setpoint);
    }

    @Override
    public void periodic() {
        super.periodic();
            for (VoltageComponent i : children) {
                if (setpoint.equals(Volts.zero())) {
                    i.neutralOutput();
                } else {
                    i.setVoltage(setpoint);
                }
        }
    }

    @Override
    public void neutralOutput() {
        setpoint = Volts.zero();
        Logger.recordOutput(getOutputLogPath("VoltageSetpoint"), setpoint);
    }
}
