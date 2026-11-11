package frc.lib.component;

import org.wpilib.units.measure.Distance;

import frc.lib.logging.LoggableComponent;

public interface DistanceSensor extends LoggableComponent {
    Distance getDistance();
}
