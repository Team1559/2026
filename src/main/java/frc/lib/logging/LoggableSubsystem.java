package frc.lib.logging;

import java.util.LinkedHashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LoggableSubsystem extends SubsystemBase {
    private final Set<LoggableComponent> children = new LinkedHashSet<>();
    private final String name;
    private final CustomLogger logger;

    protected LoggableSubsystem(String name) {
        super(name);
        this.name = name;
        logger = new CustomLogger(name);
    }

    protected final void addChildren(String folder, LoggableComponent... children) {
        for (LoggableComponent child : children) {
            if(child == null) {
                continue;
            }
            if (folder.isEmpty()) {
                child.setLogPath(getName());
            } else {
                child.setLogPath(getName() + "/" + folder);
            }
            this.children.add(child);
        }
    }

    protected final void addChildren(LoggableComponent... children) {
        addChildren("", children);
    }

    protected CustomLogger logger() {
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
