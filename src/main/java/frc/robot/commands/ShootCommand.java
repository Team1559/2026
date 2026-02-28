package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer2026;
import frc.robot.subsystems.Shooter2026;

public class ShootCommand extends Command {
    private final Shooter2026 shooter;
    private final Indexer2026 indexer;
    private final Supplier<Translation3d> targetSupplier;
    
    private boolean canShoot = false;
    public ShootCommand(Indexer2026 indexer, Shooter2026 shooter, Supplier<Translation3d> targetSupplier) {
        super();
        this.shooter = shooter;
        this.indexer = indexer;
        this.targetSupplier = targetSupplier;
        addRequirements(indexer, shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetFieldSpace(targetSupplier.get(), Translation2d.kZero);
        shooter.setSpinFlywheel(true);
        shooter.setSpinFeedwheel(false);
        indexer.stop();
        canShoot = false;
    }

    @Override
    public void execute() {
        if(canShoot){
            indexer.runForwards();
            shooter.setSpinFeedwheel(true);
        } else {
            canShoot = shooter.isFlywheelReady();
            indexer.stop();
            shooter.setSpinFeedwheel(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpinFeedwheel(false);
        shooter.setSpinFlywheel(false);
        indexer.stop();
    }

}
