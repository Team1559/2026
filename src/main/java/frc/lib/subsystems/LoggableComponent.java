package frc.lib.subsystems;

import java.util.List;

public interface LoggableComponent {
    void periodic();
    void setLogPath(String parentLogPath);
}
