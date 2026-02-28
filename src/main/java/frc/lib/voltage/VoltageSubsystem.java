package frc.lib.voltage;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.lib.loggable.LoggableSubsystem;

public class VoltageSubsystem extends LoggableSubsystem {
    private final VoltageComponent[] children;
    private Voltage targetVoltage = Volts.zero();

    public VoltageSubsystem(String name, VoltageComponent... children) {
        super(name);
        this.children = children;
        addChildren(children);
    }

    public void run(Voltage voltage) {
        this.targetVoltage = voltage;
    }

    public void stop() {
        this.targetVoltage = Volts.zero();
    }

    @Override
    public void periodic() {
        super.periodic();
        for (VoltageComponent i : children) {
            i.setVoltage(targetVoltage);
        }
    }
}
