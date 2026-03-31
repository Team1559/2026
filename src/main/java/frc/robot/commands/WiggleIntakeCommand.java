package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake2026;

public class WiggleIntakeCommand extends Command {
    private final Intake2026 intake;
    private double lastKnownTime;

    public WiggleIntakeCommand(Intake2026 intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.runForwards();
        intake.moveElbowUp();
        lastKnownTime = Timer.getTimestamp();
    }

    @Override
    public void execute() {
        double currentTime = Timer.getTimestamp();
        if (intake.isAtLowerLimit()) {
            intake.moveElbowUp();
            lastKnownTime = currentTime;
        } else if (intake.isAtUpperLimit() ||  currentTime > lastKnownTime + 0.5) {
            intake.moveElbowDown();
            lastKnownTime = currentTime;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.moveElbowDown();
        intake.stop();
    }
}
