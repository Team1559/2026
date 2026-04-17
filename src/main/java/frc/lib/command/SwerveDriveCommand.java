package frc.lib.command;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

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
        ChassisSpeeds chassisSpeeds = calculateChassisSpeeds();

        if (driveRobotOriented) {
            swerveDrive.driveRobotOriented(chassisSpeeds);
        } else if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            swerveDrive.driveFieldOriented(new ChassisSpeeds(-1 * chassisSpeeds.vxMetersPerSecond, 
                    -1 * chassisSpeeds.vyMetersPerSecond, 
                    chassisSpeeds.omegaRadiansPerSecond));
        } else {
            swerveDrive.driveFieldOriented(chassisSpeeds);
        }
    }

    //sow the seeds of chassis speeds
    protected abstract ChassisSpeeds calculateChassisSpeeds();
}
