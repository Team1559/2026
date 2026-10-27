package frc.robot.commands;

import java.util.function.Supplier;

import org.wpilib.math.geometry.Translation3d;
import org.wpilib.command2.Command;

import frc.robot.subsystems.Shooter2026;

public class ShootCommand extends Command {
    private final Shooter2026 shooter;
    private final Supplier<Translation3d> targetSupplier;
    
    private boolean canShoot = false;
    public ShootCommand(Shooter2026 shooter, Supplier<Translation3d> targetSupplier) {
        super();
        this.shooter = shooter;
        this.targetSupplier = targetSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSpinFlywheel(true);
        shooter.setShooting(false);
        canShoot = false;
    }
    
    @Override
    public void execute() {
        shooter.setTargetFieldSpace(targetSupplier.get());
        if(canShoot){
            shooter.setShooting(true);
        } else {
            canShoot = shooter.isFlywheelReady();
            shooter.setShooting(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooting(false);
        shooter.setSpinFlywheel(false);
    }

}
