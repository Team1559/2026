package frc.lib;

import java.util.LinkedHashMap;
import java.util.Map;

public abstract class LoggableIntermediate implements LoggableComponent {
    private final Map<LoggableComponent, String> children = new LinkedHashMap<>();
    private final String name;
    private String logPath;

    protected LoggableIntermediate(String name) {
        this.name = name;
    }

    protected void addChildren(String folder, LoggableComponent... children) {
        for (LoggableComponent child : children) {
            this.children.put(child, folder);
        }
    }

    protected void addChildren(LoggableComponent... children) {
        addChildren("", children);
    }

    @Override
    public final void setLogPath(String parentLogPath) {
        if (logPath != null) {
            throw new IllegalStateException("Cannot set log path more than once");
        }
        this.logPath = parentLogPath + "/" + name;
        for (LoggableComponent child : children.keySet()) {
            String folder = children.get(child);
            if (folder.isEmpty()) {
                child.setLogPath(logPath);
            } else {
                child.setLogPath(logPath + "/" + folder);
            }
        }
    }

    @Override
    public void periodic() {
        for (LoggableComponent child : children.keySet()) {
            child.periodic();
        }
    }

    protected final String getOutputLogPath(String suffix) {
        return logPath + "/Outputs/" + suffix;
    }
}
