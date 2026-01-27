package frc.lib;

public interface LoggableComponent {
    void periodic();

    void setLogPath(String parentLogPath);
}
