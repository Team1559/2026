package frc.lib.logging;

import java.util.Map;
import java.util.Map.Entry;

public class LoggableCollection implements LoggableComponent {
    private final LoggableDaycare children = new LoggableDaycare();
    private CustomLogger logger;

    protected LoggableCollection() {}

    protected final void addChildren(String folder, Map<String, ? extends LoggableComponent> children) {
        for (Entry<String, ? extends LoggableComponent> entry : children.entrySet()) {
            LoggableComponent child = entry.getValue();
            String childName = entry.getKey();

            this.children.addChild(folder, childName, child);
        }
    }

    protected final void addChild(String folder, String name, LoggableComponent child) {
        this.addChildren(folder, Map.of(name, child));
    }

    protected final void addChildren(Map<String, ? extends LoggableComponent> children) {
        this.addChildren("", children);
    }

    protected final void addChild(String name, LoggableComponent child) {
        this.addChildren("", Map.of(name, child));
    }

    @Override
    public void periodic() {
        for (LoggableComponent child : children) {
            child.periodic();
        }
    }

    @Override
    public void setLogPath(String logPath) {
        if (logger != null) {
            throw new IllegalStateException("Cannot set log path more than once");
        }
        logger = new CustomLogger(logPath);
        for (Entry<String, LoggableComponent> entry : children.getChildren().entrySet()) {
            LoggableComponent child = entry.getValue();
            String childPath = entry.getKey();
            child.setLogPath(logPath + "/" + childPath);
        }
    }

    protected final CustomLogger logger() {
        return logger;
    }
}
