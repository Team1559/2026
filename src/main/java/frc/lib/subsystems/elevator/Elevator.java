package frc.lib.subsystems.elevator;

import frc.lib.subsystems.LoggableSubsystem;
import frc.lib.subsystems.elevator.ElevatorIo.ElevatorInputs;

public class Elevator extends LoggableSubsystem {
    private final ElevatorIo io;
    private double targetPosition;

    public Elevator(String name, ElevatorIo io) {
        super(name);
        this.io = io;
        targetPosition = 0.0;
        addIo(io);
    }

    public void setTargetPosition(double pos) {
        targetPosition = pos;
        io.setTargetPosition(pos);
    }

    public void changeTargetPosition(double diff) {
        setTargetPosition(io.getInputs().currentPosition + diff);
    }

    public boolean isAtTargetPosition(double tolerance) {
        ElevatorInputs inputs = io.getInputs();
        return Math.abs(inputs.currentPosition - targetPosition) < tolerance;
    }
    
    public void stop() {
        io.stop();
    }

    public void goHome() {
        io.goHome();
        targetPosition = 0;
    }

    public boolean isHome() {
        return io.getInputs().isHome;
    }

    public double getHeight(){
        return io.getInputs().currentPosition;
    }
}
