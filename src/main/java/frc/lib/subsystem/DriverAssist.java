package frc.lib.subsystem;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.InstantCommand;

import frc.lib.logging.LoggableSubsystem;

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
