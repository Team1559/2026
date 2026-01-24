package frc.lib.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class LoggableIo<T extends LoggableInputs> implements LoggableComponent {
    private String logPath;// /path/to/logged/io
    private final T inputs;
    private final String name;

    protected LoggableIo(String name, T inputs) {
        this.name = name;
        this.inputs = inputs;
    }

    public final T getInputs() {
        return inputs;
    }

    @Override
    public void periodic() {
        logInputs();
    }

    private void logInputs() {
        if (!Logger.hasReplaySource()) {
            updateInputs(inputs);
        }
        Logger.processInputs(logPath + "/Inputs", inputs);
    }

    @Override
    public final void setLogPath(String logPath) {
        if (this.logPath != null) {
            throw new IllegalStateException("Cannot init the io twice");
        }

        this.logPath = logPath + "/" + name;
        logInputs();
    }

    protected final String getOutputLogPath(String suffix) {
        return logPath + "/Outputs/" + suffix;
    }

    protected void updateInputs(T inputs) {
        throw new UnsupportedOperationException("Use an IO implementation when not replaying from log file");
    }
}