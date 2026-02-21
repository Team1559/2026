package frc.lib.voltage;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.loggable.LoggableComponent;

public interface VoltageComponent extends LoggableComponent {
    public void setVoltage(Voltage voltage);
}
