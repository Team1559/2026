package frc.lib.subsystems;

import java.util.LinkedHashSet;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class LoggableIo<T extends LoggableInputs> {
    private final Set<LoggableIo<?>> ios = new LinkedHashSet<>();
    private String logPath;// /path/to/logged/io
    private final T inputs;
    private final String name;

    protected LoggableIo(String name, T inputs) {
        this.name = name;
        this.inputs = inputs;
    }

    protected void addIo(LoggableIo<?> io, String logSuffix) {
        io.init(logSuffix.length() == 0 ? logPath : logPath + "/" + logSuffix);
        ios.add(io);
    }

    protected void addIo(LoggableIo<?> io) {
        addIo(io, "");
    }

    public final T getInputs() {
        return inputs;
    }

    public void periodic() {
        for (LoggableIo<?> io : ios) {
            io.periodic();
        }
        log();
    }

    private void log() {
        if (!Logger.hasReplaySource()) {
            updateInputs(inputs);
        }
        Logger.processInputs(logPath + "/Inputs", inputs);
    }

    void init(String logPath) {
        if (this.logPath != null) {
            throw new IllegalStateException("Cannot init the io twice");
        }

        this.logPath = logPath + "/" + name;
        log();
    }

    protected final String getOutputLogPath(String suffix) {
        return logPath + "/Outputs/" + suffix;
    }

    protected void updateInputs(T inputs) {
        throw new UnsupportedOperationException("Use an IO implementation when not replaying from log file");
    }
}