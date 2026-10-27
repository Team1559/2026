package frc.lib.component;
import org.wpilib.units.measure.Voltage;

import frc.lib.logging.LoggableComponent;
import frc.lib.util.NeutralOutput;

public interface VoltageComponent extends LoggableComponent, NeutralOutput {
    void setVoltage(Voltage voltage);
}
