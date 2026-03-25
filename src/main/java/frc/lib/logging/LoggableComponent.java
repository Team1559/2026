package frc.lib.logging;

public interface LoggableComponent {
    void periodic();

    void setLogPath(String parentLogPath);
}
