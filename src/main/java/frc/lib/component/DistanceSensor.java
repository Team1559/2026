package frc.lib.component;

import edu.wpi.first.units.measure.Distance;

import frc.lib.logging.LoggableComponent;

public interface DistanceSensor extends LoggableComponent {
    Distance getDistance();
}
