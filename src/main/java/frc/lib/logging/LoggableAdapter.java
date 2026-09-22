package frc.lib.logging;

public class LoggableAdapter<T extends LoggableComponent> implements LoggableComponent {
    protected final T child;
    private boolean hasLogPath = false;

    protected LoggableAdapter(T child) {
        this.child = child;
    }

    @Override
    public void periodic() {
        child.periodic();
    }

    @Override
    public void setLogPath(String logPath) {
        if (hasLogPath) {
            throw new IllegalStateException("Cannot set log path more than once");
        }
        hasLogPath = true;
        child.setLogPath(logPath);
    }
}
