package frc.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.swerve.SwerveDrive;

public class StopCommand extends Command{

    private SwerveDrive drivetrain;

    public StopCommand(SwerveDrive drivetrain){
        this.drivetrain = drivetrain;
        this.addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.stopDriving();
    }
}
