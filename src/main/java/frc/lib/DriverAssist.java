package frc.lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.loggable.LoggableSubsystem;

public class DriverAssist extends LoggableSubsystem {

    public DriverAssist(String name) {
        super(name);
    }

    public void configure(Command command) {
        command.addRequirements(this);
    }

    public void cancel() {
        Command previous = CommandScheduler.getInstance().requiring(this);
        if (previous != null) {
            previous.cancel();
        }
    }

    public Command cancelCommand() {
        return new InstantCommand(this::cancel);
    }

}
