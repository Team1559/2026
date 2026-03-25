package frc.lib.logging;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.DriverStation;

@GenerateLogger
public sealed class BaseLogger permits CustomLogger {
    private static Boolean debugOverride = null;
    private String logPath;

    protected BaseLogger(String logPath) {
        this.logPath = logPath;
    }

    public static void overrideDebugMode(boolean debugMode) {
        debugOverride = debugMode;
    }

    protected static boolean shouldLogDashboard() {
        return true;
    }

    protected static boolean shouldLogDebug() {
        if (DriverStation.isFMSAttached()) {
            return false;
        }

        if (Logger.hasReplaySource()) {
            return true;
        }

        if (debugOverride != null) {
            return debugOverride;
        }

        if (DriverStation.isTest()) {
            return true;
        }

        return false;
    }

    protected String getOutputLogPath(String key) {
        return logPath + "/" + key;
    }

    protected void processInputs(LoggableInputs inputs) {
        Logger.processInputs(logPath, inputs);
    }
}
