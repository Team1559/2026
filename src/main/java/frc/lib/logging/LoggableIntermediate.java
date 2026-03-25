package frc.lib.logging;

import java.util.LinkedHashMap;
import java.util.Map;

public abstract class LoggableIntermediate implements LoggableComponent {
    private final Map<LoggableComponent, String> children = new LinkedHashMap<>();
    private final String name;
    private CustomLogger logger;

    protected LoggableIntermediate(String name) {
        this.name = name;
    }

    protected final void addChildren(String folder, LoggableComponent... children) {
        for (LoggableComponent child : children) {
            if(child == null) {
                continue;
            }
            this.children.put(child, folder);
        }
    }

    protected final void addChildren(LoggableComponent... children) {
        addChildren("", children);
    }

    @Override
    public final void setLogPath(String parentLogPath) {
        if (logger != null) {
            throw new IllegalStateException("Cannot set log path more than once");
        }
        logger = new CustomLogger(parentLogPath + "/" + name);
        for (LoggableComponent child : children.keySet()) {
            String folder = children.get(child);
            if (folder.isEmpty()) {
                child.setLogPath(parentLogPath + "/" + name);
            } else {
                child.setLogPath(parentLogPath + "/" + name + "/" + folder);
            }
        }
    }

    @Override
    public void periodic() {
        for (LoggableComponent child : children.keySet()) {
            child.periodic();
        }
    }

    protected CustomLogger logger() {
        return logger;
    }
}
