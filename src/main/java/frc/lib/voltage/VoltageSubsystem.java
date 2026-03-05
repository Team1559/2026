package frc.lib.voltage;

import static edu.wpi.first.units.Units.Volts;

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

    public void run(Voltage setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public void periodic() {
        super.periodic();
        for (VoltageComponent i : children) {
            i.setVoltage(setpoint);
        }
    }

    @Override
    public void neutralOutput() {
        setpoint = Volts.zero();   
    }
}
