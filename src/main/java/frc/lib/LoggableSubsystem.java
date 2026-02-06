package frc.lib;

import java.util.LinkedHashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LoggableSubsystem extends SubsystemBase {
    private final Set<LoggableComponent> children = new LinkedHashSet<>();

    protected LoggableSubsystem(String name) {
        super(name);
    }

    protected void addChildren(String folder, LoggableComponent... children) {
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

    protected void addChildren(LoggableComponent... children) {
        addChildren("", children);
    }

    protected String getOutputLogPath(String suffix) {
        return getName() + "/Outputs/" + suffix;
    }

    @Override
    public void periodic() {
        for (LoggableComponent child : children) {
            child.periodic();
        }
    }
}
