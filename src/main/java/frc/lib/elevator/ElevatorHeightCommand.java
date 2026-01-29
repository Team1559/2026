package frc.lib.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorHeightCommand extends Command {

    private final Elevator elevator;
    private final double height;
    private final double tolerance;

    public ElevatorHeightCommand(Elevator elevator, double height, double tolerance) {
        this.elevator = elevator;
        this.height = height;
        this.tolerance = tolerance;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetPosition(height);
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtTargetPosition(tolerance);
    }
}
