package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake2026;

public class WiggleIntakeCommand extends Command {
    private final Intake2026 intake;

    public WiggleIntakeCommand(Intake2026 intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.runForwards();
        intake.moveElbowUp();
    }

    @Override
    public void execute() {
        if (intake.isAtLowerLimit()) {
            intake.moveElbowUp();
        } else if (intake.isAtUpperLimit()) {
            intake.moveElbowDown();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.moveElbowDown();
    }
}
