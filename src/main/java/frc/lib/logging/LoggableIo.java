package frc.lib.logging;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class LoggableIo<T extends LoggableInputs> implements LoggableComponent {
    private CustomLogger logger;    
    private final T inputs;

    protected LoggableIo(T inputs) {
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
        logger.processInputs(inputs);
    }

    @Override
    public final void setLogPath(String logPath) {
        if (this.logger != null) {
            throw new IllegalStateException("Cannot init the io twice");
        }

        logger = new CustomLogger(logPath);
        logInputs();
    }

    protected final CustomLogger logger() {
        return logger;
    }

    protected void updateInputs(T inputs) {
        throw new UnsupportedOperationException("Use an IO implementation when not replaying from log file");
    }
}
