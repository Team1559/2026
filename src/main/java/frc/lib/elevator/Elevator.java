package frc.lib.elevator;

import frc.lib.LoggableSubsystem;

public class Elevator extends LoggableSubsystem {
    private final ElevatorComponent child;
    private double targetPosition;

    public Elevator(String name, ElevatorComponent child) {
        super(name);
        this.child = child;
        targetPosition = 0.0;
        addChildren(child);
    }

    public void setTargetPosition(double pos) {
        targetPosition = pos;
        child.setTargetPosition(pos);
    }

    public void changeTargetPosition(double diff) {
        setTargetPosition(child.getCurrentPosition() + diff);
    }

    public boolean isAtTargetPosition(double tolerance) {
        return Math.abs(child.getCurrentPosition() - targetPosition) < tolerance;
    }

    public void stop() {
        child.stop();
    }

    public void goHome() {
        child.goHome();
        targetPosition = 0;
    }

    public boolean isHome() {
        return child.isHome();
    }

    public double getHeight() {
        return child.getCurrentPosition();
    }
}
