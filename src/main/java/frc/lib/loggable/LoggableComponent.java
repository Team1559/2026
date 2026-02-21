package frc.lib.loggable;

public interface LoggableComponent {
    void periodic();

    void setLogPath(String parentLogPath);
}
