package frc.lib.command;

import org.wpilib.command2.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.kinematics.ChassisVelocities;

import frc.lib.subsystem.SwerveDrive;

public abstract class SwerveDriveCommand extends Command {
    protected final SwerveDrive swerveDrive;
    private final boolean driveRobotOriented;
    
    protected SwerveDriveCommand(SwerveDrive swerveDrive, boolean driveRobotOriented) {
        this.swerveDrive = swerveDrive;
        this.driveRobotOriented = driveRobotOriented;
        addRequirements(swerveDrive);
    }

    protected SwerveDriveCommand(SwerveDrive swerveDrive) {
        this(swerveDrive, false);
    }

    @Override
    public void execute() {
        ChassisVelocities chassisSpeeds = calculateChassisVelocities();

        if (driveRobotOriented) {
            swerveDrive.driveRobotOriented(chassisSpeeds);
        } else if (MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED) {
            swerveDrive.driveFieldOriented(new ChassisVelocities(-1 * chassisSpeeds.vx, 
                    -1 * chassisSpeeds.vy, 
                    chassisSpeeds.omega));
        } else {
            swerveDrive.driveFieldOriented(chassisSpeeds);
        }
    }

    //sow the seeds of chassis speeds
    protected abstract ChassisVelocities calculateChassisVelocities();
}
