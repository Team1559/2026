package frc.lib.logging;

import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;

class LoggableDaycare implements Iterable<LoggableComponent> {
    private final Map<String, LoggableComponent> children = new LinkedHashMap<>();

    final String addChild(String folder, String name, LoggableComponent child) {
        if (this.children.containsKey("")) {
            throw new IllegalStateException("Cannot add named child if unnamed child already exists");
        }
        if (child == null) {
            throw new NullPointerException("Child cannot be null");
        }

        if (name.contains("/")) {
            throw new IllegalArgumentException("Child name should not contain the character /");
        }
        if (name.isEmpty()) {
            throw new IllegalArgumentException("Child name cannot be empty");
        }

        String key;
        if (folder.isEmpty()) {
            key = name;
        } else {
            key = folder + "/" + name;
        }

        if (this.children.put(key, child) != null) {
            throw new IllegalStateException("Child already exists at path: " + key);
        }
        return key;
    }

    // loggle
    final void setChild(LoggableComponent child) {
        if (child == null) {
            throw new NullPointerException("Child cannot be null");
        }
        if (!children.isEmpty()) {
            throw new IllegalStateException("An unnamed child must be the only child");
        }
        this.children.put("", child);
    }

    final Map<String, LoggableComponent> getChildren() {
        return Collections.unmodifiableMap(children);
    }

    @Override
    public Iterator<LoggableComponent> iterator() {
        return children.values().iterator();
    }

}
