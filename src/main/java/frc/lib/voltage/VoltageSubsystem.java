package frc.lib.voltage;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.lib.loggable.LoggableSubsystem;

public class VoltageSubsystem extends LoggableSubsystem {
    private final VoltageComponent[] children;
    boolean running;
    Voltage targetVoltage;

    public VoltageSubsystem(String name, VoltageComponent... children){
        super(name);
        this.children = children;
        addChildren(children);
    }

    public void run(Voltage voltage){
        running = true;
        this.targetVoltage = voltage;
    }

    public void stop(){
        running = false;
        this.targetVoltage = Volts.of(0);
    }

    @Override
    public void periodic() {
        super.periodic();
        if (running){
            for (VoltageComponent i : children){
                i.setVoltage(targetVoltage);
            }
        } else {
            for (VoltageComponent i : children){
                i.setVoltage(Volts.of(0));
            }
        }
    }
}
