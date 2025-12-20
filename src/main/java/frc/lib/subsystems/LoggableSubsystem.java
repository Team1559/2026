package frc.lib.subsystems;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LoggableSubsystem extends SubsystemBase {
    private final Set<LoggableIo<?>> ios = new HashSet<>();

    protected LoggableSubsystem(String name) {
        super(name);
    }

    protected void addIo(LoggableIo<?> io, String logSuffix) {
        io.init(getLogPath(logSuffix));
        ios.add(io);
    }

    protected void addIo(LoggableIo<?> io){
        addIo(io, "");
    }

    protected String getLogPath(String suffix) {
        return suffix.length() == 0? getName() : getName() + "/" + suffix;
    }

    @Override
    public void periodic(){
        for (LoggableIo<?> io : ios){
            io.periodic();
        }
    }
}
