package frc.lib.logging;

import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LoggableSubsystem extends SubsystemBase {
    private final LoggableDaycare children = new LoggableDaycare();
    private final String name;
    private final CustomLogger logger;

    protected LoggableSubsystem(String name) {
        super(name);
        this.name = name;
        logger = new CustomLogger(name);
    }

    protected final void addChildren(String folder, Map<String, ? extends LoggableComponent> children) {
        for (Entry<String, ? extends LoggableComponent> entry : children.entrySet()) {
            LoggableComponent child = entry.getValue();
            String childName = entry.getKey();

            String key = this.children.addChild(folder, childName, child);
            child.setLogPath(getName() + "/" + key);
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

    protected final CustomLogger logger() {
        return logger;
    }


    @Override
    public String getName() {
        return name;
    }

    @Override
    public void periodic() {
        for (LoggableComponent child : children) {
            child.periodic();
        }
    }
}
