package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer2026;
import frc.robot.subsystems.shooter.Shooter2026;

public class ShootCommand extends Command {
    private final Shooter2026 shooter;
    private final Indexer2026 indexer;
    
    public ShootCommand(Indexer2026 indexer, Shooter2026 shooter) {
        super();
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(indexer, shooter);
    }

    @Override
    public void initialize() {
        shooter.setSpinFlywheel(true);
    }

    @Override
    public void execute() {
        if (shooter.isFlywheelReady()) {
            indexer.runForwards();
            shooter.setSpinFeedwheel(true);
        } else {
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
